#!/usr/bin/env python3
"""Square flight mission service - flies drone in a square pattern."""

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
    
    lat_rad = math.radians(origin.lat)
    d_lat = north_m / EARTH_RADIUS
    d_lon = east_m / (EARTH_RADIUS * math.cos(lat_rad))

    new_lat = origin.lat + math.degrees(d_lat)
    new_lon = origin.lon + math.degrees(d_lon)

    logger.debug(
        f"Offset: north={north_m}, east={east_m}, "
        f"Δlat={math.degrees(d_lat)}, Δlon={math.degrees(d_lon)}"
    )

    return LocationGlobalRelative(new_lat, new_lon, origin.alt)


# -----------------------------------------------------------
# SQUARE MISSION FUNCTIONS
# -----------------------------------------------------------
def hold_position(vehicle: Vehicle, seconds: float, logger: logging.Logger | None = None) -> None:
    """Hold position in LOITER mode for specified duration.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        seconds: Duration to hold position
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info("Entering LOITER...")
    vehicle.mode = VehicleMode("LOITER")

    while vehicle.mode.name != "LOITER":
        logger.info("Waiting for LOITER...")
        time.sleep(1)

    logger.info(f"Holding for {seconds} seconds...")
    for _ in range(int(seconds)):
        loc = vehicle.location.global_relative_frame
        logger.debug(f"HOLD → lat={loc.lat}, lon={loc.lon}, alt={loc.alt}")
        time.sleep(1)

    logger.info("Switching to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)


def move_offset(vehicle: Vehicle, north: float, east: float, label: str, logger: logging.Logger | None = None) -> None:
    """Move vehicle by offset in meters north/east.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        north: Meters to move north (positive) or south (negative)
        east: Meters to move east (positive) or west (negative)
        label: Label for logging (e.g., "A→B")
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info(f"Moving {label}: N={north}, E={east}")

    target = offset_location_meters(vehicle.location.global_relative_frame, north, east, logger)
    vehicle.simple_goto(target)

    for _ in range(6):
        loc = vehicle.location.global_relative_frame
        logger.debug(f"MOVE → lat={loc.lat}, lon={loc.lon}, alt={loc.alt}")
        time.sleep(1)


def execute_square_mission(
    vehicle: Vehicle,
    altitude: float,
    side_length: float,
    hold_seconds: float = 10.0,
    logger: logging.Logger | None = None,
) -> tuple[float, float]:
    """Execute a square flight path.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        altitude: Target altitude in meters
        side_length: Length of each side of the square in meters
        hold_seconds: Duration to hold at starting position before square
        logger: Optional logger instance
        
    Returns:
        Tuple of (home_lat, home_lon) after takeoff
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    # Save home position before takeoff
    home_lat = vehicle.location.global_relative_frame.lat
    home_lon = vehicle.location.global_relative_frame.lon
    logger.info(f"Home (A) saved before takeoff: lat={home_lat}, lon={home_lon}")
    
    # Takeoff
    arm_and_takeoff(vehicle, altitude, logger)
    logger.info("Takeoff complete")
    
    # Hold at starting position
    hold_position(vehicle, hold_seconds, logger)
    
    # Square path: A→B→C→D→A
    move_offset(vehicle, 0, side_length, "A→B", logger)
    move_offset(vehicle, side_length, 0, "B→C", logger)
    move_offset(vehicle, 0, -side_length, "C→D", logger)
    move_offset(vehicle, -side_length, 0, "D→A", logger)
    
    logger.info("Square mission complete")
    return home_lat, home_lon


def execute_square_mission_service(
    altitude: float = 5.0,
    side_length: float = 4.0,
    hold_seconds: float = 10.0,
) -> None:
    """Execute complete square flight mission with connection and logging.
    
    Args:
        altitude: Target altitude in meters (default: 5.0)
        side_length: Length of each side of the square in meters (default: 4.0)
        hold_seconds: Duration to hold at starting position before square (default: 10.0)
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
            logger=logger
        )
        
        # Land
        logger.info("Mission complete: Initiating LAND...")
        vehicle.mode = VehicleMode("LAND")
        
        # Wait for landing
        while vehicle.armed:
            loc = vehicle.location.global_relative_frame
            logger.info(f"Landing... alt={loc.alt:.2f}m")
            time.sleep(1)
        
        logger.info("Landing complete.")
        
    except Exception as e:
        logger.exception(f"ERROR DURING MISSION: {e}")
        if vehicle:
            emergency_slow_land(vehicle, logger)
            # Emergency landing already disconnects, so set vehicle to None
            vehicle = None
    finally:
        # Disconnect only if safe conditions are met (if not already disconnected by emergency)
        if vehicle:
            safe_disconnect(vehicle, logger)
        else:
            logger.info("PROGRAM END — No vehicle connection.")


def main() -> None:
    """CLI entry point for square mission service."""
    execute_square_mission_service()


if __name__ == "__main__":
    main()

