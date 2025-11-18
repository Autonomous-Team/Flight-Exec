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
    """Arm the vehicle, save home position, and ascend to target altitude.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        target_altitude: Target altitude in meters
        logger: Optional logger instance
        
    Returns:
        Tuple of (home_lat, home_lon, home_alt) saved before takeoff
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info("Checking pre-arm conditions...")
    while not vehicle.is_armable:
        logger.info("Waiting for vehicle to initialise (EKF/GPS)...")
        time.sleep(1)

    # Save home position BEFORE takeoff
    loc = vehicle.location.global_relative_frame
    home_lat = loc.lat
    home_lon = loc.lon
    home_alt = loc.alt
    logger.info(f"Saved HOME: lat={home_lat}, lon={home_lon}, alt={home_alt}")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        logger.info("Waiting for GUIDED...")
        time.sleep(1)

    logger.info("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        logger.info("Waiting for motors to arm...")
        time.sleep(1)

    logger.info(f"Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        logger.debug(f"Altitude: {alt:.2f}m")
        if alt >= target_altitude * 0.95:
            break
        time.sleep(1)

    logger.info("Reached target altitude.")
    return home_lat, home_lon, home_alt


def hold_position(
    vehicle: Vehicle,
    hold_time: float,
    logger: logging.Logger | None = None,
) -> None:
    """Hold position using LOITER mode for stable hover.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        hold_time: Duration to hold position in seconds
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info("Switching to LOITER for stable hover...")
    vehicle.mode = VehicleMode("LOITER")

    while vehicle.mode.name != "LOITER":
        logger.info("Waiting for LOITER...")
        time.sleep(1)

    logger.info(f"Holding position for {hold_time} seconds...")
    for _ in range(int(hold_time)):
        loc = vehicle.location.global_relative_frame
        logger.debug(f"HOLD → lat={loc.lat}, lon={loc.lon}, alt={loc.alt:.2f}m")
        time.sleep(1)

    logger.info("Hold complete. Switching back to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        logger.info("Waiting for GUIDED...")
        time.sleep(1)


def land_at_home(
    vehicle: Vehicle,
    home_lat: float,
    home_lon: float,
    logger: logging.Logger | None = None,
) -> None:
    """Return to home position and land safely.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        home_lat: Home latitude
        home_lon: Home longitude
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info("Returning to home before landing...")

    home_point = LocationGlobalRelative(home_lat, home_lon, 2)
    vehicle.simple_goto(home_point)

    for _ in range(6):
        loc = vehicle.location.global_relative_frame
        logger.debug(f"Returning home → lat={loc.lat}, lon={loc.lon}, alt={loc.alt:.2f}m")
        time.sleep(1)

    logger.info("Initiating LAND mode...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        logger.debug(f"Landing... Alt={alt:.2f}m")
        time.sleep(1)

    logger.info("Landed and disarmed.")


def execute_takeoff_land(
    target_altitude: float = 5.0,
    hold_seconds: float = 10.0,
) -> None:
    """Execute complete takeoff, hold, and landing sequence.
    
    Args:
        target_altitude: Target altitude in meters (default: 5.0)
        hold_seconds: Duration to hold position in seconds (default: 10.0)
    """
    # Setup logger
    logger = setup_logger()
    
    # Silence verbose dronekit logs
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)
    
    vehicle: Vehicle | None = None
    
    try:
        # Free busy ports
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)
        
        # Connect to flight controller
        vehicle, port, baud = connect_to_first_available(candidate_ports)
        logger.info(f"Connected to FC on {port}@{baud}")
        
        # Enable MAVLink message logging
        mavlink_logger = create_mavlink_logger(logger)
        vehicle.add_message_listener('*', mavlink_logger)
        logger.info("Full MAVLink logging enabled.")
        
        # Takeoff and save home position
        home_lat, home_lon, home_alt = arm_and_takeoff(vehicle, target_altitude, logger)
        
        # Hold using LOITER
        hold_position(vehicle, hold_seconds, logger)
        
        # Return home and land safely
        land_at_home(vehicle, home_lat, home_lon, logger)
        
    except Exception as e:
        logger.exception(f"ERROR: {e}")
        if vehicle:
            emergency_slow_land(vehicle, logger)
    
    # Disconnect only if safe conditions are met
    if vehicle:
        safe_disconnect(vehicle, logger)
    else:
        logger.info("PROGRAM COMPLETE — No vehicle connection.")


def main() -> None:
    """CLI entry point for takeoff/land routine."""
    execute_takeoff_land()


if __name__ == "__main__":
    main()
