#!/usr/bin/env python3
"""High-level takeoff and landing operations with home position tracking."""

from __future__ import annotations

import collections

# Fix dronekit compatibility for Python 3.10+
if not hasattr(collections, "MutableMapping"):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

import logging
import time

from dronekit import LocationGlobalRelative, Vehicle, VehicleMode

from utils.detect_controller import connect_to_first_available, list_candidate_ports
from utils.emergency_land import emergency_slow_land
from utils.free_ports import free_acm_usb_ports
from utils.logger import setup_logger
from utils.mavlink_logger import create_mavlink_logger
from utils.safe_disconnect import safe_disconnect

# --------------------------------------------------------------------------
# WAIT-UNTIL HELPER (15s global wait) - exits early if condition recovers
# --------------------------------------------------------------------------
def wait_until(predicate_fn, timeout_s=15, period_s=1.0, logger: logging.Logger | None = None, msg: str = "") -> bool:
    """
    Repeatedly evaluate predicate_fn() until it returns True or timeout elapses.
    Returns True if predicate became True, False if timed out.
    Exceptions during predicate are swallowed (treated as not-yet-true).
    """
    if logger:
        logger.info(f"[WAIT] {msg} (timeout={timeout_s}s)")

    start = time.time()
    while time.time() - start < timeout_s:
        try:
            if predicate_fn():
                if logger:
                    logger.info(f"[WAIT] Condition met: {msg}")
                return True
        except Exception:
            # swallow transient read errors and keep waiting
            pass

        if logger:
            elapsed = int(time.time() - start)
            logger.info(f"[WAIT] Waiting... {elapsed}/{int(timeout_s)}s - {msg}")
        time.sleep(period_s)

    if logger:
        logger.warning(f"[WAIT] Timeout reached ({timeout_s}s): {msg}")
    return False


# -----------------------------------------------------------
# HIGH-LEVEL TAKEOFF / LAND UTILITIES (unchanged flow)
# -----------------------------------------------------------
def arm_and_takeoff(
    vehicle: Vehicle,
    target_altitude: float,
    logger: logging.Logger | None = None,
) -> tuple[float, float, float]:
    if logger is None:
        logger = logging.getLogger("drone_logger")

    try:
        logger.info("Checking pre-arm conditions...")
        while not vehicle.is_armable:
            logger.info("Waiting for vehicle to initialise (EKF/GPS)...")
            time.sleep(1)

        loc = vehicle.location.global_relative_frame
        home_lat, home_lon, home_alt = loc.lat, loc.lon, loc.alt
        logger.info(f"Saved HOME: lat={home_lat}, lon={home_lon}, alt={home_alt}")

        # Try to set GUIDED
        try:
            vehicle.mode = VehicleMode("GUIDED")
        except Exception as e:
            logger.error(f"Unable to set GUIDED: {e} → attempting recovery before emergency land")
            ok = wait_until(lambda: getattr(vehicle, "mode", None) is not None and vehicle.mode.name == "GUIDED",
                            timeout_s=15, period_s=1, logger=logger, msg="Recover GUIDED mode")
            if not ok:
                logger.error("Unable to set GUIDED after wait → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise
            # else: recovered, continue

        # Wait for GUIDED (exit early if it becomes GUIDED)
        if not wait_until(lambda: vehicle.mode is not None and vehicle.mode.name == "GUIDED",
                          timeout_s=15, period_s=1, logger=logger, msg="Waiting for GUIDED..."):
            logger.error("GUIDED mode not reached within wait → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise RuntimeError("GUIDED timeout")

        logger.info("Arming motors...")
        vehicle.armed = True

        # Wait for arming (30s loop replaced by early-exit wait)
        armed_ok = wait_until(lambda: getattr(vehicle, "armed", False) is True,
                              timeout_s=15, period_s=1, logger=logger, msg="Waiting for motors to arm")
        if not armed_ok:
            logger.error("Motors failed to arm after wait → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise RuntimeError("Failed to arm")

        logger.info(f"Taking off to {target_altitude} meters...")
        try:
            vehicle.simple_takeoff(target_altitude)
        except Exception as e:
            logger.error(f"simple_takeoff failed: {e} → attempting recovery before emergency land")
            # Wait for altitude telemetry to show progress (or for mode/armed changes)
            ok = wait_until(
                lambda: (
                    getattr(vehicle, "location", None) is not None
                    and getattr(vehicle.location.global_relative_frame, "alt", None) is not None
                    and vehicle.location.global_relative_frame.alt >= target_altitude * 0.95
                ),
                timeout_s=15,
                period_s=1,
                logger=logger,
                msg="Recovering simple_takeoff (waiting for altitude)"
            )
            if not ok:
                logger.error("simple_takeoff unrecoverable → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise

        # Wait up to original takeoff loop duration (120s) for altitude,
        # but if an altitude read error occurs we will attempt a 15s recovery before emergency.
        reached = False
        for _ in range(120):
            try:
                alt = vehicle.location.global_relative_frame.alt
                logger.debug(f"Altitude: {alt:.2f}m")
                if alt is not None and alt >= target_altitude * 0.95:
                    reached = True
                    break
            except Exception as e:
                logger.error(f"Altitude read error: {e} → attempting recovery before emergency land")
                ok = wait_until(
                    lambda: getattr(vehicle, "location", None) is not None and
                    getattr(vehicle.location.global_relative_frame, "alt", None) is not None,
                    timeout_s=15,
                    period_s=1,
                    logger=logger,
                    msg="Waiting for altitude telemetry"
                )
                if not ok:
                    logger.error("Altitude telemetry unrecoverable → EMERGENCY LAND")
                    emergency_slow_land(vehicle, logger)
                    raise
            time.sleep(1)

        if not reached:
            # After original takeoff wait, give a final 15s recovery window
            logger.error("Takeoff not reached in allotted time → attempting final recovery before emergency land")
            ok = wait_until(lambda: getattr(vehicle.location.global_relative_frame, "alt", 0) >= target_altitude * 0.95,
                            timeout_s=15, period_s=1, logger=logger, msg="Final recovery waiting for altitude")
            if not ok:
                logger.error("Takeoff timeout → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise RuntimeError("Takeoff failed")

        logger.info("Reached target altitude.")
        return home_lat, home_lon, home_alt

    except Exception:
        emergency_slow_land(vehicle, logger)
        raise


def hold_position(
    vehicle: Vehicle,
    hold_time: float,
    logger: logging.Logger | None = None,
) -> None:
    if logger is None:
        logger = logging.getLogger("drone_logger")

    try:
        logger.info("Switching to LOITER for stable hover...")
        try:
            vehicle.mode = VehicleMode("LOITER")
        except Exception as e:
            logger.error(f"Cannot set LOITER: {e} → attempting recovery before emergency land")
            ok = wait_until(lambda: getattr(vehicle, "mode", None) is not None and vehicle.mode.name == "LOITER",
                            timeout_s=15, period_s=1, logger=logger, msg="Recover LOITER mode")
            if not ok:
                logger.error("LOITER set failed after wait → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise

        # Wait for LOITER mode (up to 10s originally, but we keep that and then allow 15s recovery)
        start = time.time()
        while vehicle.mode.name != "LOITER":
            logger.info("Waiting for LOITER...")
            time.sleep(1)
            if time.time() - start > 10:
                logger.error("LOITER mode initial timeout → attempting recovery")
                ok = wait_until(lambda: getattr(vehicle, "mode", None) is not None and vehicle.mode.name == "LOITER",
                                timeout_s=15, period_s=1, logger=logger, msg="Trying to recover LOITER")
                if not ok:
                    logger.error("LOITER mode unrecoverable → EMERGENCY LAND")
                    emergency_slow_land(vehicle, logger)
                    raise RuntimeError("LOITER timeout")
                break

        logger.info(f"Holding position for {hold_time} seconds...")
        for _ in range(int(hold_time)):
            try:
                loc = vehicle.location.global_relative_frame
                logger.debug(f"HOLD → lat={loc.lat}, lon={loc.lon}, alt={loc.alt:.2f}m")
            except Exception as e:
                logger.error(f"Location read failed during hold: {e} → attempting recovery before emergency land")
                ok = wait_until(lambda: getattr(vehicle, "location", None) is not None and
                                getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
                                timeout_s=15, period_s=1, logger=logger, msg="Recover location during hold")
                if not ok:
                    logger.error("Location unavailable → EMERGENCY LAND")
                    emergency_slow_land(vehicle, logger)
                    raise
            time.sleep(1)

        logger.info("Hold complete. Switching back to GUIDED...")

        try:
            vehicle.mode = VehicleMode("GUIDED")
        except Exception as e:
            logger.error(f"Cannot set GUIDED: {e} → attempting recovery before emergency land")
            ok = wait_until(lambda: getattr(vehicle, "mode", None) is not None and vehicle.mode.name == "GUIDED",
                            timeout_s=15, period_s=1, logger=logger, msg="Recover GUIDED mode")
            if not ok:
                logger.error("GUIDED set failed after wait → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise

        start = time.time()
        while vehicle.mode.name != "GUIDED":
            logger.info("Waiting for GUIDED...")
            time.sleep(1)
            if time.time() - start > 10:
                logger.error("GUIDED mode initial timeout → attempting recovery")
                ok = wait_until(lambda: getattr(vehicle, "mode", None) is not None and vehicle.mode.name == "GUIDED",
                                timeout_s=15, period_s=1, logger=logger, msg="Trying to recover GUIDED")
                if not ok:
                    logger.error("GUIDED mode unrecoverable → EMERGENCY LAND")
                    emergency_slow_land(vehicle, logger)
                    raise RuntimeError("GUIDED timeout")

    except Exception:
        emergency_slow_land(vehicle, logger)
        raise


def land_at_home(
    vehicle: Vehicle,
    home_lat: float,
    home_lon: float,
    logger: logging.Logger | None = None,
) -> None:
    if logger is None:
        logger = logging.getLogger("drone_logger")

    try:
        logger.info("Returning to home before landing...")

        try:
            home_point = LocationGlobalRelative(home_lat, home_lon, 2)
        except Exception as e:
            logger.error(f"Failed to create home point: {e} → attempting recovery before emergency land")
            ok = wait_until(lambda: getattr(vehicle, "location", None) is not None,
                            timeout_s=15, period_s=1, logger=logger, msg="Recover vehicle connection for home point")
            if not ok:
                logger.error("Home point creation unrecoverable → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise

        try:
            vehicle.simple_goto(home_point)
        except Exception as e:
            logger.error(f"Failed goto HOME: {e} → attempting recovery before emergency land")
            ok = wait_until(lambda: getattr(vehicle, "location", None) is not None and
                            getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
                            timeout_s=15, period_s=1, logger=logger, msg="Recover goto HOME")
            if not ok:
                logger.error("Failed goto HOME after wait → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise

        for _ in range(8):
            try:
                loc = vehicle.location.global_relative_frame
                logger.debug(f"Returning home → lat={loc.lat}, lon={loc.lon}, alt={loc.alt:.2f}m")
            except Exception as e:
                logger.error(f"Location read failed: {e} → attempting recovery before emergency land")
                ok = wait_until(lambda: getattr(vehicle, "location", None) is not None and
                                getattr(vehicle.location.global_relative_frame, "lat", None) is not None,
                                timeout_s=15, period_s=1, logger=logger, msg="Recover location during return")
                if not ok:
                    logger.error("Location read unrecoverable → EMERGENCY LAND")
                    emergency_slow_land(vehicle, logger)
                    raise
            time.sleep(1)

        logger.info("Initiating LAND mode...")
        try:
            vehicle.mode = VehicleMode("LAND")
        except Exception as e:
            logger.error(f"Cannot set LAND: {e} → attempting recovery before emergency land")
            ok = wait_until(lambda: getattr(vehicle, "mode", None) is not None and vehicle.mode.name == "LAND",
                            timeout_s=15, period_s=1, logger=logger, msg="Recover LAND mode set")
            if not ok:
                logger.error("LAND set failed after wait → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise

        for _ in range(300):
            try:
                alt = vehicle.location.global_relative_frame.alt
                logger.debug(f"Landing... Alt={alt:.2f}m")
            except Exception:
                logger.debug("Landing... Alt unknown")

            if not vehicle.armed:
                logger.info("Landed and disarmed.")
                break

            time.sleep(1)
        else:
            logger.error("Landing timeout → attempting recovery before emergency land")
            ok = wait_until(lambda: not getattr(vehicle, "armed", True),
                            timeout_s=15, period_s=1, logger=logger, msg="Final recovery waiting for disarm")
            if not ok:
                logger.error("Landing timeout → EMERGENCY LAND AGAIN")
                emergency_slow_land(vehicle, logger)

    except Exception:
        emergency_slow_land(vehicle, logger)
        raise


def execute_takeoff_land(
    target_altitude: float = 5.0,
    hold_seconds: float = 10.0,
) -> None:
    logger = setup_logger()
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)

    vehicle: Vehicle | None = None

    try:
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)

        vehicle, port, baud = connect_to_first_available(candidate_ports)
        logger.info(f"Connected to FC on {port}@{baud}")

        mavlink_logger = create_mavlink_logger(logger)
        vehicle.add_message_listener('*', mavlink_logger)

        home_lat, home_lon, home_alt = arm_and_takeoff(vehicle, target_altitude, logger)

        hold_position(vehicle, hold_seconds, logger)

        land_at_home(vehicle, home_lat, home_lon, logger)

    except Exception as e:
        logger.exception(f"ERROR: {e}")
        if vehicle:
            emergency_slow_land(vehicle, logger)
            vehicle = None

    if vehicle:
        safe_disconnect(vehicle, logger)
    else:
        logger.info("PROGRAM COMPLETE — No vehicle connection.")


def main() -> None:
    execute_takeoff_land()


if __name__ == "__main__":
    main()
