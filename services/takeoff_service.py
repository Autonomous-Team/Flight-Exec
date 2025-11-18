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

        try:
            vehicle.mode = VehicleMode("GUIDED")
        except Exception as e:
            logger.error(f"Unable to set GUIDED: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        while vehicle.mode.name != "GUIDED":
            logger.info("Waiting for GUIDED...")
            time.sleep(1)

        logger.info("Arming motors...")
        vehicle.armed = True
        for _ in range(30):
            if vehicle.armed:
                break
            logger.info("Waiting for motors to arm...")
            time.sleep(1)
        else:
            logger.error("Motors failed to arm → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise RuntimeError("Failed to arm")

        logger.info(f"Taking off to {target_altitude} meters...")
        try:
            vehicle.simple_takeoff(target_altitude)
        except Exception as e:
            logger.error(f"simple_takeoff failed: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        for _ in range(120):
            try:
                alt = vehicle.location.global_relative_frame.alt
                logger.debug(f"Altitude: {alt:.2f}m")
                if alt >= target_altitude * 0.95:
                    break
            except Exception as e:
                logger.error(f"Altitude read error: {e} → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise

            time.sleep(1)
        else:
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
            logger.error(f"Cannot set LOITER: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        start = time.time()
        while vehicle.mode.name != "LOITER":
            logger.info("Waiting for LOITER...")
            time.sleep(1)
            if time.time() - start > 10:
                logger.error("LOITER timeout → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise RuntimeError("LOITER timeout")

        logger.info(f"Holding position for {hold_time} seconds...")
        for _ in range(int(hold_time)):
            try:
                loc = vehicle.location.global_relative_frame
                logger.debug(f"HOLD → lat={loc.lat}, lon={loc.lon}, alt={loc.alt:.2f}m")
            except Exception as e:
                logger.error(f"Location read failed during hold: {e} → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise
            time.sleep(1)

        logger.info("Hold complete. Switching back to GUIDED...")

        try:
            vehicle.mode = VehicleMode("GUIDED")
        except Exception as e:
            logger.error(f"Cannot set GUIDED: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        start = time.time()
        while vehicle.mode.name != "GUIDED":
            logger.info("Waiting for GUIDED...")
            time.sleep(1)
            if time.time() - start > 10:
                logger.error("GUIDED timeout → EMERGENCY LAND")
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
            logger.error(f"Failed to create home point: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        try:
            vehicle.simple_goto(home_point)
        except Exception as e:
            logger.error(f"Failed goto HOME: {e} → EMERGENCY LAND")
            emergency_slow_land(vehicle, logger)
            raise

        for _ in range(8):
            try:
                loc = vehicle.location.global_relative_frame
                logger.debug(f"Returning home → lat={loc.lat}, lon={loc.lon}, alt={loc.alt:.2f}m")
            except Exception as e:
                logger.error(f"Location read failed: {e} → EMERGENCY LAND")
                emergency_slow_land(vehicle, logger)
                raise
            time.sleep(1)

        logger.info("Initiating LAND mode...")
        try:
            vehicle.mode = VehicleMode("LAND")
        except Exception as e:
            logger.error(f"Cannot set LAND: {e} → EMERGENCY LAND")
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
