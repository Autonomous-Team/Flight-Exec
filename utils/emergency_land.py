#!/usr/bin/env python3
"""Mission service for point-to-point navigation with LOITER hold and return-to-home."""

from __future__ import annotations

import collections

# Fix dronekit compatibility for Python 3.10+
if not hasattr(collections, "MutableMapping"):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

import logging
import math
import time

from dronekit import LocationGlobalRelative, Vehicle, VehicleMode

from services.takeoff_service import arm_and_takeoff
from utils.detect_controller import connect_to_first_available, list_candidate_ports
from utils.emergency_land import emergency_slow_land
from utils.free_ports import free_acm_usb_ports
from utils.logger import setup_logger
from utils.mavlink_logger import create_mavlink_logger
from utils.safe_disconnect import safe_disconnect

# -----------------------------------------------------------
# NAVIGATION UTILITIES
# -----------------------------------------------------------
EARTH_RADIUS = 6378137.0


def get_location_offset_meters(
    origin: LocationGlobalRelative,
    d_north: float,
    d_east: float,
) -> LocationGlobalRelative:
    lat_rad = math.radians(origin.lat)
    d_lat = d_north / EARTH_RADIUS
    d_lon = d_east / (EARTH_RADIUS * math.cos(lat_rad))
    new_lat = origin.lat + math.degrees(d_lat)
    new_lon = origin.lon + math.degrees(d_lon)
    return LocationGlobalRelative(new_lat, new_lon, origin.alt)


def haversine_distance_m(
    location_a: LocationGlobalRelative,
    location_b: LocationGlobalRelative,
) -> float:
    lat1 = math.radians(location_a.lat)
    lon1 = math.radians(location_a.lon)
    lat2 = math.radians(location_b.lat)
    lon2 = math.radians(location_b.lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    sa = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(sa), math.sqrt(max(0.0, 1 - sa)))
    return EARTH_RADIUS * c


# -----------------------------------------------------------
# MISSION FUNCTIONS (FULLY PATCHED WITH SAFETY FALLBACKS)
# -----------------------------------------------------------
def goto_point(
    vehicle: Vehicle,
    target_location: LocationGlobalRelative,
    groundspeed: float = 0.5,
    tolerance_m: float = 0.5,
    logger: logging.Logger | None = None,
) -> None:
    if logger is None:
        logger = logging.getLogger("drone_logger")

    logger.info(
        f"Navigating to target: lat={target_location.lat}, "
        f"lon={target_location.lon}, alt={target_location.alt:.2f}m"
    )

    try:
        vehicle.simple_goto(target_location, groundspeed=groundspeed)
    except Exception as e:
        logger.error(f"simple_goto failed: {e} → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    while True:
        try:
            current = vehicle.location.global_relative_frame
            dist = haversine_distance_m(current, target_location)
            logger.debug(f"Distance to target: {dist:.2f}m")

            if dist <= tolerance_m:
                logger.info("Reached target location (within tolerance).")
                break
        except Exception as e:
            logger.error(f"Location read error during navigation: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        time.sleep(1)


def hold_at_point_using_loiter(
    vehicle: Vehicle,
    hold_time: float = 10.0,
    logger: logging.Logger | None = None,
) -> None:
    if logger is None:
        logger = logging.getLogger("drone_logger")

    logger.info("Switching to LOITER for stable hold...")

    try:
        vehicle.mode = VehicleMode("LOITER")
    except Exception as e:
        logger.error(f"Cannot set LOITER: {e} → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    start = time.time()
    while vehicle.mode.name != "LOITER":
        logger.info("Waiting for LOITER...")
        time.sleep(1)
        if time.time() - start > 10:
            logger.error("LOITER mode timeout → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

    logger.info(f"Holding in LOITER for {hold_time} seconds...")

    t0 = time.time()
    while time.time() - t0 < hold_time:
        try:
            loc = vehicle.location.global_relative_frame
            logger.debug(f"LOITER HOLD → lat={loc.lat}, lon={loc.lon}, alt={loc.alt:.2f}m")
        except Exception as e:
            logger.error(f"Location read failed during LOITER: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        time.sleep(1)

    logger.info("Switching back to GUIDED...")

    try:
        vehicle.mode = VehicleMode("GUIDED")
    except Exception as e:
        logger.error(f"Cannot switch to GUIDED: {e} → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    start = time.time()
    while vehicle.mode.name != "GUIDED":
        logger.info("Waiting for GUIDED mode...")
        time.sleep(1)
        if time.time() - start > 10:
            logger.error("GUIDED mode timeout → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise


def return_home_and_land(
    vehicle: Vehicle,
    home_location: LocationGlobalRelative,
    logger: logging.Logger | None = None,
) -> None:
    if logger is None:
        logger = logging.getLogger("drone_logger")

    if home_location is None:
        logger.error("HOME location unknown → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        return

    logger.info("Returning to HOME in GUIDED mode (approach altitude 2m)...")

    try:
        home_approach = LocationGlobalRelative(home_location.lat, home_location.lon, 2)
    except Exception as e:
        logger.error(f"Home approach creation failed: {e} → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    goto_point(vehicle, home_approach, groundspeed=0.5, tolerance_m=0.7, logger=logger)

    logger.info("Commanding LAND mode...")

    try:
        vehicle.mode = VehicleMode("LAND")
    except Exception as e:
        logger.error(f"Cannot set LAND mode: {e} → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    start = time.time()
    while vehicle.armed:
        try:
            alt = vehicle.location.global_relative_frame.alt
            logger.debug(f"Landing... alt={alt:.2f}m")
        except Exception:
            logger.debug("Landing... alt unknown")

        time.sleep(1)

        if time.time() - start > 300:
            logger.error("Timeout waiting for landing → EMERGENCY LAND AGAIN")
            emergency_slow_land(vehicle, logger)
            break

    logger.info("Landing sequence complete (if disarm occurred).")


def execute_point_to_point_mission(
    target_altitude: float = 3.0,
    move_north: float = 5.0,
    move_east: float = 0.0,
    hold_time: float = 10.0,
) -> None:
    logger = setup_logger()
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)

    vehicle: Vehicle | None = None
    home_location: LocationGlobalRelative | None = None

    try:
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)

        vehicle, port, baud = connect_to_first_available(candidate_ports)
        logger.info(f"Connected to FC on {port}@{baud}")

        mavlink_logger = create_mavlink_logger(logger)
        vehicle.add_message_listener('*', mavlink_logger)

        home_lat, home_lon, home_alt = arm_and_takeoff(vehicle, target_altitude, logger)
        home_location = LocationGlobalRelative(home_lat, home_lon, home_alt)

        point_B = get_location_offset_meters(home_location, move_north, move_east)
        point_B.alt = target_altitude

        goto_point(vehicle, point_B, groundspeed=0.5, tolerance_m=0.5, logger=logger)
        hold_at_point_using_loiter(vehicle, hold_time=hold_time, logger=logger)
        return_home_and_land(vehicle, home_location, logger)

        logger.info("Mission complete.")

    except Exception as e:
        logger.exception(f"ERROR during mission: {e}")
        if vehicle:
            emergency_slow_land(vehicle, logger)
            vehicle = None

    if vehicle:
        safe_disconnect(vehicle, logger)
    else:
        logger.info("Script finished — No vehicle connection.")


def main() -> None:
    execute_point_to_point_mission()


if __name__ == "__main__":
    main()
