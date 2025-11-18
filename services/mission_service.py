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
    """Convert meter offsets to LocationGlobalRelative.
    
    Uses spherical approximation (accurate for small distances).
    
    Args:
        origin: Starting location
        d_north: Meters to move north (positive) or south (negative)
        d_east: Meters to move east (positive) or west (negative)
        
    Returns:
        New LocationGlobalRelative with offset coordinates
    """
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
    """Calculate Haversine distance between two locations in meters.
    
    Args:
        location_a: First location
        location_b: Second location
        
    Returns:
        Distance in meters
    """
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
# MISSION FUNCTIONS
# -----------------------------------------------------------
def goto_point(
    vehicle: Vehicle,
    target_location: LocationGlobalRelative,
    groundspeed: float = 0.5,
    tolerance_m: float = 0.5,
    logger: logging.Logger | None = None,
) -> None:
    """Navigate to a target location using GUIDED mode.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        target_location: Target location to navigate to
        groundspeed: Ground speed in m/s (default: 0.5)
        tolerance_m: Distance tolerance in meters (default: 0.5)
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info(
        f"Navigating to target: lat={target_location.lat}, "
        f"lon={target_location.lon}, alt={target_location.alt:.2f}m"
    )
    try:
        vehicle.simple_goto(target_location, groundspeed=groundspeed)
    except Exception as e:
        logger.warning(f"simple_goto raised: {e}")

    while True:
        try:
            current = vehicle.location.global_relative_frame
            dist = haversine_distance_m(current, target_location)
            logger.debug(f"Distance to target: {dist:.2f}m")
            if dist <= tolerance_m:
                logger.info("Reached target location (within tolerance).")
                break
        except Exception as e:
            logger.warning(f"Error reading location while navigating: {e}")
        time.sleep(1)


def hold_at_point_using_loiter(
    vehicle: Vehicle,
    hold_time: float = 10.0,
    logger: logging.Logger | None = None,
) -> None:
    """Hold position using LOITER mode for stable hover.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        hold_time: Duration to hold position in seconds (default: 10.0)
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info("Switching to LOITER for stable hold...")
    try:
        vehicle.mode = VehicleMode("LOITER")
    except Exception as e:
        logger.warning(f"Could not set LOITER: {e}")

    start = time.time()
    while vehicle.mode.name != "LOITER":
        logger.info("Waiting for LOITER...")
        time.sleep(1)
        if time.time() - start > 10:
            logger.warning("LOITER taking a while; continuing to wait.")

    logger.info(f"Holding in LOITER for {hold_time} seconds...")
    t0 = time.time()
    while time.time() - t0 < hold_time:
        try:
            loc = vehicle.location.global_relative_frame
            logger.debug(f"LOITER HOLD → lat={loc.lat}, lon={loc.lon}, alt={loc.alt:.2f}m")
        except Exception as e:
            logger.warning(f"Error reading location during LOITER: {e}")
        time.sleep(1)

    logger.info("LOITER hold complete. Switching back to GUIDED.")
    try:
        vehicle.mode = VehicleMode("GUIDED")
    except Exception as e:
        logger.warning(f"Could not switch back to GUIDED: {e}")
    
    # Wait until GUIDED active
    start = time.time()
    while vehicle.mode.name != "GUIDED":
        logger.info("Waiting for GUIDED mode...")
        time.sleep(1)
        if time.time() - start > 10:
            logger.warning("GUIDED switch taking long; continuing to wait.")


def return_home_and_land(
    vehicle: Vehicle,
    home_location: LocationGlobalRelative,
    logger: logging.Logger | None = None,
) -> None:
    """Return to home position and land safely.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        home_location: Home location to return to
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    if home_location is None:
        logger.error("HOME_LOCATION unknown — cannot return home safely.")
        return

    logger.info("Returning to HOME in GUIDED mode (approach altitude 2m)...")
    home_approach = LocationGlobalRelative(home_location.lat, home_location.lon, 2)
    goto_point(vehicle, home_approach, groundspeed=0.5, tolerance_m=0.7, logger=logger)

    logger.info("Within approach tolerance; commanding LAND mode.")
    try:
        vehicle.mode = VehicleMode("LAND")
    except Exception as e:
        logger.warning(f"Could not set LAND mode: {e}")

    logger.info("Waiting for disarm (landing complete)...")
    start = time.time()
    while vehicle.armed:
        try:
            alt = vehicle.location.global_relative_frame.alt
            logger.debug(f"Landing... alt={alt:.2f}m")
        except Exception:
            logger.debug("Landing... (no location)")
        time.sleep(1)
        if time.time() - start > 300:
            logger.error("Timeout waiting for disarm while landing.")
            break
    logger.info("Landed and disarmed (if disarm occurred).")


def execute_point_to_point_mission(
    target_altitude: float = 3.0,
    move_north: float = 5.0,
    move_east: float = 0.0,
    hold_time: float = 10.0,
) -> None:
    """Execute complete point-to-point mission: takeoff, move to B, hold, return home.
    
    Args:
        target_altitude: Takeoff altitude in meters (default: 3.0)
        move_north: Meters to move north from home (default: 5.0)
        move_east: Meters to move east from home (default: 0.0)
        hold_time: Duration to hold at point B in seconds (default: 10.0)
    """
    # Setup logger
    logger = setup_logger()
    
    # Silence verbose dronekit logs
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)
    
    vehicle: Vehicle | None = None
    home_location: LocationGlobalRelative | None = None
    
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
        logger.info("MAVLink full logging enabled.")
        
        # Takeoff and save home position
        home_lat, home_lon, home_alt = arm_and_takeoff(vehicle, target_altitude, logger)
        home_location = LocationGlobalRelative(home_lat, home_lon, home_alt)
        logger.info(
            f"Saved HOME before takeoff: lat={home_location.lat}, "
            f"lon={home_location.lon}, alt={home_location.alt:.2f}m"
        )
        
        # Compute point B relative to home
        point_B = get_location_offset_meters(home_location, move_north, move_east)
        point_B.alt = target_altitude  # Maintain altitude
        logger.info(f"Point B calculated: lat={point_B.lat}, lon={point_B.lon}, alt={point_B.alt:.2f}m")
        
        # Navigate to point B using GUIDED
        goto_point(vehicle, point_B, groundspeed=0.5, tolerance_m=0.5, logger=logger)
        
        # Hold at B using LOITER
        hold_at_point_using_loiter(vehicle, hold_time=hold_time, logger=logger)
        
        # Return home and land
        return_home_and_land(vehicle, home_location, logger)
        
        logger.info("Mission complete.")
        
    except Exception as e:
        logger.exception(f"ERROR during mission: {e}")
        if vehicle:
            emergency_slow_land(vehicle, logger)
            # Emergency landing already disconnects, so set vehicle to None
            vehicle = None
        else:
            logger.critical("Vehicle object is None — cannot run emergency procedure.")
    
    # Disconnect only if safe conditions are met (if not already disconnected by emergency)
    if vehicle:
        safe_disconnect(vehicle, logger)
    else:
        logger.info("Script finished — No vehicle connection.")


def main() -> None:
    """CLI entry point for point-to-point mission."""
    execute_point_to_point_mission()


if __name__ == "__main__":
    main()
