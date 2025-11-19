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


def wait_until(predicate_fn, timeout, period=1.0):
    """Repeatedly check a condition; exit early on success, timeout otherwise."""
    start = time.time()
    while True:
        try:
            if predicate_fn():
                return
        except Exception:
            pass

        if time.time() - start >= timeout:
            raise TimeoutError("wait_until() timed out")

        time.sleep(period)


def get_location_offset_meters(origin, d_north, d_east):
    lat_rad = math.radians(origin.lat)
    d_lat = d_north / EARTH_RADIUS
    d_lon = d_east / (EARTH_RADIUS * math.cos(lat_rad))
    new_lat = origin.lat + math.degrees(d_lat)
    new_lon = origin.lon + math.degrees(d_lon)
    return LocationGlobalRelative(new_lat, new_lon, origin.alt)


def haversine_distance_m(a, b):
    lat1 = math.radians(a.lat)
    lon1 = math.radians(a.lon)
    lat2 = math.radians(b.lat)
    lon2 = math.radians(b.lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    sa = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(sa), math.sqrt(max(0.0, 1 - sa)))
    return EARTH_RADIUS * c


# -----------------------------------------------------------
# MISSION FUNCTIONS WITH wait_until()
# -----------------------------------------------------------
def goto_point(vehicle, target_location, groundspeed=0.3, tolerance_m=0.5, logger=None):
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

    try:
        wait_until(
            lambda: haversine_distance_m(vehicle.location.global_relative_frame, target_location)
            <= tolerance_m,
            timeout=15,
            period=1,
        )
    except TimeoutError:
        logger.error("goto_point(): Position not improving within timeout → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    logger.info("Reached target location (within tolerance).")


def hold_at_point_using_loiter(vehicle, hold_time=10.0, logger=None):
    if logger is None:
        logger = logging.getLogger("drone_logger")

    logger.info("Switching to LOITER...")

    try:
        vehicle.mode = VehicleMode("LOITER")
    except Exception as e:
        logger.error(f"Cannot set LOITER: {e} → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    # Wait for LOITER mode (with early exit)
    try:
        wait_until(lambda: vehicle.mode.name == "LOITER", timeout=10, period=1)
    except TimeoutError:
        logger.error("LOITER mode timeout → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    logger.info(f"Holding in LOITER for {hold_time} seconds...")

    start = time.time()
    while time.time() - start < hold_time:
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

    # Wait for GUIDED mode (with early exit)
    try:
        wait_until(lambda: vehicle.mode.name == "GUIDED", timeout=10, period=1)
    except TimeoutError:
        logger.error("GUIDED mode timeout → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise


def return_home_and_land(vehicle, home_location, target_altitude, logger=None):
    if logger is None:
        logger = logging.getLogger("drone_logger")

    if home_location is None:
        logger.error("HOME location unknown → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        return

    logger.info(f"Returning to HOME at {target_altitude}m altitude (maintaining mission altitude)...")

    # Return to home at the same altitude as the mission (not descending)
    home_approach = LocationGlobalRelative(home_location.lat, home_location.lon, target_altitude)

    goto_point(vehicle, home_approach, groundspeed=0.3, tolerance_m=0.7, logger=logger)

    logger.info("Commanding LAND...")

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

    logger.info("Landing sequence complete.")


def execute_point_to_point_mission(target_altitude=5.0, move_north=5.0, move_east=0.0, hold_time=10.0):
    logger = setup_logger()
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)

    vehicle = None
    home_location = None

    try:
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)

        vehicle, port, baud = connect_to_first_available(candidate_ports)
        logger.info(f"Connected to FC on {port}@{baud}")

        mavlink_logger = create_mavlink_logger(logger)
        vehicle.add_message_listener('*', mavlink_logger)

        logger.info(f"Mission altitude set to {target_altitude}m - will maintain this altitude throughout the mission")
        
        home_lat, home_lon, home_alt = arm_and_takeoff(vehicle, target_altitude, logger)
        # Use target_altitude for home_location to ensure consistent altitude throughout mission
        home_location = LocationGlobalRelative(home_lat, home_lon, target_altitude)

        point_B = get_location_offset_meters(home_location, move_north, move_east)
        point_B.alt = target_altitude

        logger.info(f"Navigating to point B at {target_altitude}m altitude...")
        goto_point(vehicle, point_B, groundspeed=0.3, tolerance_m=0.5, logger=logger)
        
        logger.info(f"Holding at point B in LOITER mode at {target_altitude}m altitude...")
        hold_at_point_using_loiter(vehicle, hold_time=hold_time, logger=logger)
        
        logger.info(f"Returning to home at {target_altitude}m altitude...")
        return_home_and_land(vehicle, home_location, target_altitude, logger)

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


def main():
    execute_point_to_point_mission()


if __name__ == "__main__":
    main()
