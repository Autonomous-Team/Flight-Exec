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
# HELPER: wait_until for recoverable conditions (15 seconds)
# -----------------------------------------------------------
def wait_until(predicate_fn, timeout_s=15, period_s=1.0, logger: logging.Logger | None = None, msg: str = "") -> bool:
    """
    Repeatedly evaluate predicate_fn() until True or timeout_s elapses.
    Returns True if predicate became True, False if timed out.
    Exceptions during predicate are swallowed and treated as 'not yet true'.
    """
    if logger:
        logger.info(f"[WAIT] {msg} (timeout={timeout_s}s)")
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            if predicate_fn():
                if logger:
                    logger.info(f"[WAIT] Condition satisfied: {msg}")
                return True
        except Exception:
            # swallow transient read exceptions and continue waiting
            pass
        if logger:
            elapsed = int(time.time() - start)
            logger.info(f"[WAIT] Waiting... {elapsed}/{int(timeout_s)}s - {msg}")
        time.sleep(period_s)
    if logger:
        logger.warning(f"[WAIT] Timeout reached ({timeout_s}s): {msg}")
    return False


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
        logger.error(f"Invalid origin location provided: {e} → attempting 15s recovery for valid origin")
        ok = wait_until(
            lambda: getattr(origin, "lat", None) is not None and getattr(origin, "lon", None) is not None,
            timeout_s=15,
            period_s=1,
            logger=logger,
            msg="Waiting for valid origin lat/lon"
        )
        if not ok:
            logger.error("Invalid origin still present after wait → EMERGENCY LAND")
            emergency_slow_land(None, logger)  # no vehicle object here; caller should handle, but call with None to log
            raise RuntimeError("Invalid origin location")
        lat_rad = math.radians(origin.lat)

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
    groundspeed: float = 0.3,
    logger: logging.Logger | None = None,
    retries: int = 3,
    retry_delay: float = 1.0,
) -> None:
    """Call simple_goto with retries; if all retries fail attempt 15s recoverable wait, then raise."""
    if logger is None:
        logger = logging.getLogger("drone_logger")

    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            vehicle.simple_goto(target, groundspeed=groundspeed)
            logger.debug(f"simple_goto issued (attempt {attempt}) → target lat={target.lat}, lon={target.lon}, alt={target.alt}")
            return
        except Exception as e:
            last_exc = e
            logger.warning(f"simple_goto attempt {attempt} failed: {e}")
            time.sleep(retry_delay)

    # After retries, attempt recoverable wait (sensor/telemetry recovery) for 15s
    logger.error(f"simple_goto failed after {retries} attempts: {last_exc} → attempting 15s recoverable wait")
    ok = wait_until(
        lambda: getattr(vehicle, "location", None) is not None and getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
        timeout_s=15,
        period_s=1,
        logger=logger,
        msg="Recovering vehicle telemetry for simple_goto"
    )
    if not ok:
        logger.error("simple_goto unrecoverable after wait → caller will handle emergency")
        raise last_exc
    # else recovered — try one last time
    try:
        vehicle.simple_goto(target, groundspeed=groundspeed)
        logger.debug("simple_goto issued after recovery")
        return
    except Exception as e:
        logger.error(f"simple_goto still failing after recovery attempt: {e}")
        raise


def _safe_read_location(
    vehicle: Vehicle,
    logger: logging.Logger | None = None,
    retries: int = 3,
    retry_delay: float = 0.5,
) -> LocationGlobalRelative:
    """Read vehicle.location.global_relative_frame with retries; if still invalid, attempt 15s recoverable wait then raise."""
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

    logger.error(f"Location read failed after {retries} attempts: {last_exc} → attempting 15s recoverable wait")
    ok = wait_until(
        lambda: getattr(vehicle, "location", None) is not None and getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
        timeout_s=15,
        period_s=1,
        logger=logger,
        msg="Waiting for valid location data"
    )
    if not ok:
        logger.error("Location read unrecoverable after wait")
        raise last_exc if last_exc is not None else RuntimeError("Location read failed")
    # If recovered, try one final read
    loc = vehicle.location.global_relative_frame
    if loc is None or loc.lat is None or loc.lon is None or loc.alt is None:
        logger.error("Location still invalid after recovery wait")
        raise RuntimeError("Location invalid after recovery")
    return loc


def hold_position(vehicle: Vehicle, seconds: float, logger: logging.Logger | None = None) -> None:
    """Hold position in LOITER mode for specified duration."""
    if logger is None:
        logger = logging.getLogger("drone_logger")

    try:
        logger.info("Entering LOITER...")
        try:
            vehicle.mode = VehicleMode("LOITER")
        except Exception as e:
            # Mode set failures are considered non-recoverable per your preference (Option A)
            logger.error(f"Failed to set LOITER: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        # wait with timeout (mode transition). This is a mode-set related wait; do not convert to recoverable 15s.
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
            except Exception as e:
                logger.error(f"Location read failed during hold: {e} → attempting 15s recoverable wait")
                ok = wait_until(
                    lambda: getattr(vehicle, "location", None) is not None and getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
                    timeout_s=15, period_s=1, logger=logger, msg="Recover location during hold"
                )
                if not ok:
                    logger.error("Location read unrecoverable during hold → EMERGENCY LAND")
                    emergency_slow_land(vehicle, logger)
                    raise
                # else recovered — continue loop
            time.sleep(1)

        logger.info("Switching to GUIDED...")
        try:
            vehicle.mode = VehicleMode("GUIDED")
        except Exception as e:
            # Mode-set failure — treat as non-recoverable (immediate emergency)
            logger.error(f"Cannot set GUIDED: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        start = time.time()
        while vehicle.mode.name != "GUIDED":
            if time.time() - start > 15:
                logger.error("GUIDED entry timeout → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise RuntimeError("GUIDED timeout")
            logger.info("Waiting for GUIDED...")
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
    groundspeed: float = 0.3,
    goto_retries: int = 3,
    goto_delay: float = 1.0,
) -> None:
    """Move vehicle by offset in meters north/east with retries then emergency land."""
    if logger is None:
        logger = logging.getLogger("drone_logger")

    logger.info(f"Moving {label}: N={north}, E={east}")

    try:
        # Read current location robustly (sensor recovery handled inside)
        try:
            origin = _safe_read_location(vehicle, logger=logger)
        except Exception as e:
            logger.error(f"Unable to read origin location before move: {e} → attempting 15s recoverable wait")
            ok = wait_until(
                lambda: getattr(vehicle, "location", None) is not None and getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
                timeout_s=15,
                period_s=1,
                logger=logger,
                msg="Recover origin location before move"
            )
            if not ok:
                logger.error("Unable to read origin after wait → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise
            origin = _safe_read_location(vehicle, logger=logger)  # try again

        target = offset_location_meters(origin, north, east, logger=logger)
        target.alt = origin.alt  # keep same altitude unless caller overrides

        # Attempt simple_goto with retries and recoverable wait inside _safe_simple_goto
        try:
            _safe_simple_goto(vehicle, target, groundspeed=groundspeed, logger=logger, retries=goto_retries, retry_delay=goto_delay)
        except Exception as e:
            logger.error(f"simple_goto failed for move_offset: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        # Wait for the vehicle to reach approximate area (use robust location reads)
        for _ in range(12):  # ~12 seconds observation
            try:
                loc = _safe_read_location(vehicle, logger=logger)
                logger.debug(f"MOVE → lat={loc.lat}, lon={loc.lon}, alt={loc.alt}")
            except Exception as e:
                logger.error(f"Location read failed during move: {e} → attempting 15s recoverable wait")
                ok = wait_until(
                    lambda: getattr(vehicle, "location", None) is not None and getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
                    timeout_s=15,
                    period_s=1,
                    logger=logger,
                    msg="Recover location during move"
                )
                if not ok:
                    logger.error("Location read unrecoverable during move → EMERGENCY LAND")
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
    except Exception as e:
        logger.error(f"Unable to read home position before takeoff: {e} → attempting 15s recoverable wait")
        ok = wait_until(
            lambda: getattr(vehicle, "location", None) is not None and getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
            timeout_s=15,
            period_s=1,
            logger=logger,
            msg="Recover home position before takeoff"
        )
        if not ok:
            logger.error("Unable to read home position after wait → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise
        origin = _safe_read_location(vehicle, logger=logger)
        home_lat = origin.lat
        home_lon = origin.lon
        logger.info(f"Home (A) saved after recovery: lat={home_lat}, lon={home_lon}")

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
    side_length: float = 7.0,
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
            # Mode-set failures are non-recoverable per Option A
            logger.error(f"Cannot set LAND: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            vehicle = None
            return

        # Wait for landing with timeout (300s). If that times out, attempt a short 15s recoverable wait for disarm,
        # otherwise emergency land.
        start = time.time()
        while vehicle and getattr(vehicle, "armed", True):
            try:
                loc = _safe_read_location(vehicle, logger=logger)
                logger.info(f"Landing... alt={loc.alt:.2f}m")
            except Exception:
                logger.debug("Landing... alt unknown")
            if time.time() - start > 300:
                logger.error("Landing timeout → attempting final 15s recoverable wait for disarm")
                ok = wait_until(lambda: not getattr(vehicle, "armed", True),
                                timeout_s=15, period_s=1, logger=logger, msg="Waiting for disarm during final recovery")
                if not ok:
                    logger.error("Final recovery failed → EMERGENCY LAND AGAIN")
                    emergency_slow_land(vehicle, logger)
                    vehicle = None
                    break
                else:
                    logger.info("Disarmed during final recovery wait")
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
