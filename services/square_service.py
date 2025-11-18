#!/usr/bin/env python3
"""Square flight mission service - flies drone in a square pattern (safely patched)."""

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
# LOCATION UTILITIES (SQUARE-SPECIFIC)
# -----------------------------------------------------------
EARTH_RADIUS = 6378137.0


def offset_location_meters(
    origin: LocationGlobalRelative,
    north_m: float,
    east_m: float,
    logger: logging.Logger | None = None,
) -> LocationGlobalRelative:
    """Calculate a new location offset by meters north/east from origin.

    Args:
        origin: Starting location
        north_m: Meters to move north (positive) or south (negative)
        east_m: Meters to move east (positive) or west (negative)
        logger: Optional logger instance

    Returns:
        New LocationGlobalRelative with offset coordinates
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")

    try:
        lat_rad = math.radians(origin.lat)
    except Exception as e:
        logger.error(f"Invalid origin location provided: {e} → EMERGENCY LAND")
        raise

    d_lat = north_m / EARTH_RADIUS
    d_lon = east_m / (EARTH_RADIUS * math.cos(lat_rad))

    new_lat = origin.lat + math.degrees(d_lat)
    new_lon = origin.lon + math.degrees(d_lon)

    logger.debug(
        f"Offset: north={north_m}, east={east_m}, "
        f"Δlat={math.degrees(d_lat):.8f}, Δlon={math.degrees(d_lon):.8f}"
    )

    return LocationGlobalRelative(new_lat, new_lon, origin.alt)


# -----------------------------------------------------------
# SQUARE MISSION FUNCTIONS (PATCHED, RETRIES BEFORE EMERGENCY)
# -----------------------------------------------------------
def _safe_simple_goto(
    vehicle: Vehicle,
    target: LocationGlobalRelative,
    groundspeed: float | None = None,
    logger: logging.Logger | None = None,
    retries: int = 3,
    retry_delay: float = 1.0,
) -> None:
    """Call simple_goto with retries; raise after retries exhausted."""
    if logger is None:
        logger = logging.getLogger("drone_logger")

    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            if groundspeed is not None:
                vehicle.simple_goto(target, groundspeed=groundspeed)
            else:
                vehicle.simple_goto(target)
            logger.debug(f"simple_goto issued (attempt {attempt}) → target lat={target.lat}, lon={target.lon}, alt={target.alt}")
            return
        except Exception as e:
            last_exc = e
            logger.warning(f"simple_goto attempt {attempt} failed: {e}")
            time.sleep(retry_delay)

    logger.error(f"simple_goto failed after {retries} attempts: {last_exc}")
    raise last_exc  # caller will handle emergency landing


def _safe_read_location(
    vehicle: Vehicle,
    logger: logging.Logger | None = None,
    retries: int = 3,
    retry_delay: float = 0.5,
) -> LocationGlobalRelative:
    """Read vehicle.location.global_relative_frame with retries; raise if unavailable."""
    if logger is None:
        logger = logging.getLogger("drone_logger")

    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            loc = vehicle.location.global_relative_frame
            # Some vehicles may return None or incomplete; validate
            if loc is None or loc.lat is None or loc.lon is None or loc.alt is None:
                raise RuntimeError("Incomplete location data")
            return loc
        except Exception as e:
            last_exc = e
            logger.warning(f"Location read attempt {attempt} failed: {e}")
            time.sleep(retry_delay)

    logger.error(f"Location read failed after {retries} attempts: {last_exc}")
    raise last_exc


def hold_position(vehicle: Vehicle, seconds: float, logger: logging.Logger | None = None) -> None:
    """Hold position in LOITER mode for specified duration."""
    if logger is None:
        logger = logging.getLogger("drone_logger")

    try:
        logger.info("Entering LOITER...")
        try:
            vehicle.mode = VehicleMode("LOITER")
        except Exception as e:
            logger.error(f"Failed to set LOITER: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        # wait with timeout
        start = time.time()
        while vehicle.mode.name != "LOITER":
            if time.time() - start > 15:
                logger.error("LOITER entry timeout → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise RuntimeError("LOITER timeout")
            logger.info("Waiting for LOITER...")
            time.sleep(1)

        logger.info(f"Holding for {seconds} seconds...")
        end_time = time.time() + float(seconds)
        while time.time() < end_time:
            try:
                loc = _safe_read_location(vehicle, logger=logger)
                logger.debug(f"HOLD → lat={loc.lat}, lon={loc.lon}, alt={loc.alt}")
            except Exception:
                logger.error("Location read failed during hold → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise
            time.sleep(1)

        logger.info("Switching to GUIDED...")
        try:
            vehicle.mode = VehicleMode("GUIDED")
        except Exception as e:
            logger.error(f"Cannot set GUIDED: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        start = time.time()
        while vehicle.mode.name != "GUIDED":
            if time.time() - start > 15:
                logger.error("GUIDED entry timeout → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise RuntimeError("GUIDED timeout")
            time.sleep(1)

    except Exception:
        # ensure emergency if unexpected error bubbles up
        emergency_slow_land(vehicle, logger)
        raise


def move_offset(
    vehicle: Vehicle,
    north: float,
    east: float,
    label: str,
    logger: logging.Logger | None = None,
    groundspeed: float | None = None,
    goto_retries: int = 3,
    goto_delay: float = 1.0,
) -> None:
    """Move vehicle by offset in meters north/east with retries then emergency land."""
    if logger is None:
        logger = logging.getLogger("drone_logger")

    logger.info(f"Moving {label}: N={north}, E={east}")

    try:
        # Read current location robustly
        try:
            origin = _safe_read_location(vehicle, logger=logger)
        except Exception:
            logger.error("Unable to read origin location before move → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        target = offset_location_meters(origin, north, east, logger=logger)
        target.alt = origin.alt  # keep same altitude unless caller overrides

        # Attempt simple_goto with retries
        try:
            _safe_simple_goto(vehicle, target, groundspeed=groundspeed, logger=logger, retries=goto_retries, retry_delay=goto_delay)
        except Exception:
            logger.error("simple_goto failed for move_offset → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        # Wait for the vehicle to reach approximate area (use haversine-like quick check)
        for _ in range(12):  # ~12 seconds observation
            try:
                loc = _safe_read_location(vehicle, logger=logger)
                logger.debug(f"MOVE → lat={loc.lat}, lon={loc.lon}, alt={loc.alt}")
            except Exception:
                logger.error("Location read failed during move → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise
            time.sleep(1)

    except Exception:
        # ensure emergency handled by callers; re-raise for higher-level handling
        raise


def execute_square_mission(
    vehicle: Vehicle,
    altitude: float,
    side_length: float,
    hold_seconds: float = 10.0,
    logger: logging.Logger | None = None,
) -> tuple[float, float]:
    """Execute a square flight path."""
    if logger is None:
        logger = logging.getLogger("drone_logger")

    # Save home position before takeoff
    try:
        origin = _safe_read_location(vehicle, logger=logger)
        home_lat = origin.lat
        home_lon = origin.lon
        logger.info(f"Home (A) saved before takeoff: lat={home_lat}, lon={home_lon}")
    except Exception:
        logger.error("Unable to read home position before takeoff → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    # Takeoff
    try:
        arm_and_takeoff(vehicle, altitude, logger)
        logger.info("Takeoff complete")
    except Exception:
        logger.error("Takeoff failed → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    # Hold at starting position
    hold_position(vehicle, hold_seconds, logger)

    # Square path: A→B→C→D→A (use retries before emergency)
    try:
        move_offset(vehicle, 0, side_length, "A→B", logger=logger)
        move_offset(vehicle, side_length, 0, "B→C", logger=logger)
        move_offset(vehicle, 0, -side_length, "C→D", logger=logger)
        move_offset(vehicle, -side_length, 0, "D→A", logger=logger)
    except Exception:
        logger.error("One of the legs failed → EMERGENCY LAND")
        emergency_slow_land(vehicle, logger)
        raise

    logger.info("Square mission complete")
    return home_lat, home_lon


def execute_square_mission_service(
    altitude: float = 5.0,
    side_length: float = 4.0,
    hold_seconds: float = 10.0,
) -> None:
    """Execute complete square flight mission with connection and logging."""
    logger = setup_logger()
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)

    vehicle: Vehicle | None = None

    try:
        # Free busy ports
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)

        # Connect to flight controller
        vehicle, port, baud = connect_to_first_available(candidate_ports)
        logger.info(f"Connected to FC on {port}@{baud}")

        # Enable raw FC message logging
        mavlink_logger = create_mavlink_logger(logger)
        vehicle.add_message_listener('*', mavlink_logger)
        logger.info("Full MAVLink telemetry logging enabled.")

        # Execute square mission
        logger.info("Starting square mission...")
        home_lat, home_lon = execute_square_mission(
            vehicle,
            altitude=altitude,
            side_length=side_length,
            hold_seconds=hold_seconds,
            logger=logger,
        )

        # Land
        logger.info("Mission complete: Initiating LAND...")
        try:
            vehicle.mode = VehicleMode("LAND")
        except Exception as e:
            logger.error(f"Cannot set LAND: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            vehicle = None
            return

        # Wait for landing with timeout
        start = time.time()
        while vehicle and getattr(vehicle, "armed", True):
            try:
                loc = _safe_read_location(vehicle, logger=logger)
                logger.info(f"Landing... alt={loc.alt:.2f}m")
            except Exception:
                logger.debug("Landing... alt unknown")
            if time.time() - start > 300:
                logger.error("Landing timeout → EMERGENCY LAND AGAIN")
                emergency_slow_land(vehicle, logger)
                vehicle = None
                break
            time.sleep(1)

        logger.info("Landing complete.")

    except Exception as e:
        logger.exception(f"ERROR DURING MISSION: {e}")
        if vehicle:
            try:
                emergency_slow_land(vehicle, logger)
            except Exception:
                logger.exception("Emergency landing attempt failed during exception handling.")
            vehicle = None
    finally:
        # Disconnect only if safe conditions are met (if not already disconnected by emergency)
        if vehicle:
            try:
                safe_disconnect(vehicle, logger)
            except Exception as e:
                logger.error(f"Error during safe_disconnect: {e}")
        else:
            logger.info("PROGRAM END — No vehicle connection.")


def main() -> None:
    """CLI entry point for square mission service."""
    execute_square_mission_service()


if __name__ == "__main__":
    main()
