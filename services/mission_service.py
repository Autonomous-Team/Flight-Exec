#!/usr/bin/env python3
"""Mission service for waypoint navigation flows with comprehensive safety monitoring."""

from __future__ import annotations

import collections
import logging
import threading
import time
from typing import Optional

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import LocationGlobalRelative, Vehicle, VehicleMode

from services.enhanced_takeoff_service import arm_and_takeoff, land_and_disarm
from services.battery_monitor import start_battery_monitor
from services.heartbeat_monitor import start_heartbeat_monitor
from services.jetson_monitor import start_jetson_monitor
from services.fc_telemetry_monitor import start_fc_telemetry_monitor
from services.safety_manager import (
    get_safe_landing_reason,
    is_safe_landing_triggered,
    safe_landing,
)
from utils.detect_controller import connect_to_first_available, list_candidate_ports
from utils.geometry_utils import haversine_m, location_offset_meters
from utils.logging_utils import setup_logging
from utils.port_utils import report_port_users

if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore


def select_active_gps(vehicle: Vehicle, wait_seconds: int = 10) -> int:
    """
    Return the index of the GPS source with a valid fix.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        wait_seconds: Maximum time to wait for a GPS fix if none is available
    
    Returns:
        Index of the GPS source with valid fix
    
    Raises:
        RuntimeError: If no GPS fix is detected within the wait period
    """
    print("\n[CHECK] Scanning GPS sources...")
    logging.info("Scanning GPS sources...")
    
    gps_sources = []
    for idx in range(2):
        gps_attr = getattr(vehicle, f"gps_{idx}", None)
        if gps_attr:
            fix_type = getattr(gps_attr, "fix_type", None)
            satellites_visible = getattr(gps_attr, "satellites_visible", None)
            if fix_type is not None and satellites_visible is not None:
                gps_sources.append((idx, fix_type, satellites_visible))
                print(
                    f"  → GPS{idx + 1}: fix_type={fix_type}, "
                    f"satellites={satellites_visible}"
                )
                logging.info("GPS%d: fix_type=%d, satellites=%d", idx + 1, fix_type, satellites_visible)

    gps_sources = [entry for entry in gps_sources if entry[1] >= 3]
    if gps_sources:
        best_idx, fix_type, satellites = max(gps_sources, key=lambda item: item[2])
        print(f"✅ Selected GPS{best_idx + 1} (fix_type={fix_type}, satellites={satellites})")
        logging.info("Selected GPS%d (fix_type=%d, satellites=%d)", best_idx + 1, fix_type, satellites)
        return best_idx

    print(f"❌ No GPS fix yet. Waiting for up to {wait_seconds} seconds for lock...")
    logging.warning("No GPS fix yet. Waiting for up to %s seconds", wait_seconds)
    
    for i in range(wait_seconds):
        for idx in range(2):
            gps_attr = getattr(vehicle, f"gps_{idx}", None)
            if gps_attr:
                fix_type = getattr(gps_attr, "fix_type", None)
                satellites_visible = getattr(gps_attr, "satellites_visible", None)
                if fix_type is not None and fix_type >= 3:
                    print(
                        f"✅ Using GPS{idx + 1} "
                        f"(fix_type={fix_type}, satellites={satellites_visible})"
                    )
                    logging.info("Using GPS%d (fix_type=%d, satellites=%d)", idx + 1, fix_type, satellites_visible)
                    return idx
        time.sleep(1)

    raise RuntimeError(f"❌ No GPS fix within {wait_seconds} seconds.")


def goto_point(
    vehicle: Vehicle,
    d_north: float,
    d_east: float,
    target_altitude: float,
    *,
    groundspeed: Optional[float] = None,
    arrival_threshold_m: Optional[float] = None,
) -> bool:
    """
    Fly to a point offset from the current position with safety checks.
    
    Args:
        vehicle: Connected vehicle instance
        d_north: Meters to travel north (negative for south)
        d_east: Meters to travel east (negative for west)
        target_altitude: Target altitude in meters
        groundspeed: Ground speed in m/s
    
    Returns:
        True if successful, False if safety trigger occurred
    """
    if groundspeed is None or arrival_threshold_m is None:
        from config import get_mission_config

        cfg = get_mission_config()
        if groundspeed is None:
            groundspeed = cfg.groundspeed
        if arrival_threshold_m is None:
            arrival_threshold_m = cfg.goto_threshold_m

    print(
        f"\n[STEP 5] Going to target point ({d_north} m North, {d_east} m East) "
        f"at {target_altitude} m altitude ({groundspeed} m/s)..."
    )
    logging.info(
        "Going to target point (%.2f m North, %.2f m East) at %.2f m altitude (%.2f m/s)",
        d_north,
        d_east,
        target_altitude,
        groundspeed,
    )

    # Get current location
    current_location = vehicle.location.global_relative_frame
    current_lat = getattr(current_location, "lat", None)
    current_lon = getattr(current_location, "lon", None)
    current_alt = getattr(current_location, "alt", 0.0) or 0.0

    if current_lat is None or current_lon is None:
        logging.error("Cannot get current location for goto_point")
        raise RuntimeError("❌ Cannot get current location")

    # Calculate target location using geometry utils
    target_lat, target_lon = location_offset_meters(current_lat, current_lon, d_north, d_east)
    target_location = LocationGlobalRelative(target_lat, target_lon, target_altitude)

    logging.debug(
        "Goto target: lat=%.6f lon=%.6f alt=%.2f (from lat=%.6f lon=%.6f)",
        target_lat,
        target_lon,
        target_altitude,
        current_lat,
        current_lon,
    )

    try:
        vehicle.simple_goto(target_location, groundspeed=groundspeed)
        logging.info("Sent simple_goto command to target location")
    except Exception as e:
        logging.exception("simple_goto failed: %s", e)
        raise RuntimeError(f"❌ simple_goto failed: {e}")

    # Wait until we reach the target
    while True:
        # Check for safety triggers
        if is_safe_landing_triggered():
            reason = get_safe_landing_reason()
            logging.warning("Safe landing triggered during goto: %s", reason)
            return False

        # Check connection
        if not getattr(vehicle, "connected", False):
            logging.error("Vehicle disconnected during goto")
            return False

        # Get current position
        current_location = vehicle.location.global_relative_frame
        current_lat = getattr(current_location, "lat", None)
        current_lon = getattr(current_location, "lon", None)
        current_alt = getattr(current_location, "alt", 0.0) or 0.0

        if current_lat is None or current_lon is None:
            logging.warning("No valid GPS fix during goto")
            time.sleep(1)
            continue

        # Calculate distance using haversine
        distance = haversine_m(current_lat, current_lon, target_lat, target_lon)
        alt_diff = abs(current_alt - target_altitude)

        print(f"  → Distance: {distance:.2f} m | Altitude: {current_alt:.2f} m")
        logging.debug("Goto progress: distance=%.2f m, alt_diff=%.2f m", distance, alt_diff)

        if distance <= arrival_threshold_m and alt_diff <= 0.2:
            print("✅ Arrived at target point and altitude")
            logging.info("Arrived at target point and altitude")
            break

        time.sleep(1)

    return True


def execute_point_to_point_mission(
    takeoff_altitude: float,
    north_offset: float,
    east_offset: float,
    target_altitude: float,
    *,
    hover_delay: Optional[float] = None,
    gps_wait_seconds: Optional[int] = None,
    enable_safety_monitoring: bool = True,
) -> None:
    """
    Run a complete point-to-point mission from takeoff to landing with safety monitoring.
    
    Args:
        takeoff_altitude: Altitude to reach on takeoff (meters)
        north_offset: Meters to travel north from the takeoff point
        east_offset: Meters to travel east from the takeoff point
        target_altitude: Cruise altitude at the waypoint (meters)
        hover_delay: Seconds to loiter at waypoint before landing. If None, uses config.
        gps_wait_seconds: Seconds to wait for GPS fix if none available. If None, uses config.
        enable_safety_monitoring: If True, enable all safety monitoring threads
    """
    from config import get_mission_config

    cfg = get_mission_config()
    
    # Use config values if not provided
    if hover_delay is None:
        hover_delay = cfg.hover_delay
    if gps_wait_seconds is None:
        gps_wait_seconds = cfg.gps_wait_seconds

    # Setup logging with config
    setup_logging(detailed=cfg.logging_detailed, log_dir=cfg.logging_dir, log_prefix="mission")

    vehicle: Optional[Vehicle] = None
    hb_stop = threading.Event()
    battery_thread: Optional[threading.Thread] = None
    hb_thread: Optional[threading.Thread] = None
    jetson_thread: Optional[threading.Thread] = None
    fc_telemetry_thread: Optional[threading.Thread] = None

    try:
        # Report port usage
        report_port_users()

        # Connect to vehicle
        candidate_ports = list_candidate_ports()
        vehicle, port, baud = connect_to_first_available(candidate_ports)
        print(f"📡 Connected on {port} @ {baud}")
        logging.info("Connected on %s @ %s", port, baud)

        # Start safety monitoring if enabled
        if enable_safety_monitoring:
            if cfg.enable_heartbeat_monitor:
                hb_thread = start_heartbeat_monitor(
                    vehicle,
                    hb_stop,
                    warn_seconds=cfg.heartbeat_warn_sec,
                    critical_seconds=cfg.heartbeat_critical_sec,
                )
            if cfg.enable_battery_monitor:
                battery_thread = start_battery_monitor(
                    vehicle,
                    hb_stop,
                    critical_threshold=cfg.battery_critical_threshold,
                    low_threshold=cfg.battery_low_threshold,
                )
            if cfg.enable_jetson_monitor:
                jetson_thread = start_jetson_monitor(hb_stop)
            if cfg.enable_fc_telemetry_monitor:
                fc_telemetry_thread = start_fc_telemetry_monitor(vehicle, hb_stop)
            logging.info("All safety monitoring threads started")

        # Select active GPS
        try:
            active_gps_index = select_active_gps(vehicle, wait_seconds=gps_wait_seconds)
            print(f"📍 Active GPS: GPS{active_gps_index + 1}")
        except Exception as e:
            logging.exception("GPS selection failed: %s", e)
            raise

        # Arm and takeoff (uses enhanced_takeoff_service which saves home position)
        try:
            arm_and_takeoff(
                vehicle,
                takeoff_altitude,
                home_wait_timeout=cfg.home_wait_timeout,
            )
            # Check for safety triggers after takeoff
            if is_safe_landing_triggered():
                reason = get_safe_landing_reason()
                print(f"\n🚨 Safe landing triggered after takeoff: {reason}")
                safe_landing(vehicle, reason)
                return
        except Exception as e:
            logging.exception("Takeoff failed: %s", e)
            safe_landing(vehicle, f"Takeoff failed: {e}")
            return

        # Navigate to waypoint
        try:
            goto_ok = goto_point(
                vehicle,
                north_offset,
                east_offset,
                target_altitude,
                groundspeed=cfg.groundspeed,
                arrival_threshold_m=cfg.goto_threshold_m,
            )
            if not goto_ok:
                print("\n⚠️ Goto interrupted by safety trigger")
                logging.warning("Goto interrupted by safety trigger")
                safe_landing(vehicle, "Goto interrupted by safety trigger")
                return
        except Exception as e:
            logging.exception("Goto failed: %s", e)
            safe_landing(vehicle, f"Goto failed: {e}")
            return

        # Hover at waypoint
        if hover_delay > 0:
            print(f"\n[HOVER] Holding position for {hover_delay:.0f} seconds...")
            logging.info("Holding position for %s seconds", hover_delay)
            
            start_time = time.time()
            while time.time() - start_time < hover_delay:
                if is_safe_landing_triggered():
                    reason = get_safe_landing_reason()
                    print(f"\n🚨 Safe landing triggered during hover: {reason}")
                    safe_landing(vehicle, reason)
                    return
                if not getattr(vehicle, "connected", False):
                    logging.error("Vehicle disconnected during hover")
                    safe_landing(vehicle, "Vehicle disconnected during hover")
                    return
                time.sleep(0.5)

        # Land and disarm
        try:
            land_and_disarm(vehicle)
        except Exception as e:
            logging.exception("Landing failed: %s", e)
            safe_landing(vehicle, f"Landing failed: {e}")

    except KeyboardInterrupt:
        print("\n✋ KeyboardInterrupt received. Initiating safe landing.")
        logging.warning("KeyboardInterrupt received. Initiating safe landing.")
        if vehicle is not None:
            safe_landing(vehicle, "KeyboardInterrupt - user requested")
    except Exception as exc:
        print(f"\n❌ Unhandled error: {exc}")
        logging.exception("Unhandled error in execute_point_to_point_mission: %s", exc)
        if vehicle is not None:
            safe_landing(vehicle, f"Unhandled exception: {exc}")
    finally:
        # Stop monitoring threads
        if enable_safety_monitoring:
            hb_stop.set()
            threads_to_join = [
                ("Heartbeat", hb_thread),
                ("Battery", battery_thread),
                ("Jetson", jetson_thread),
                ("FC Telemetry", fc_telemetry_thread),
            ]

            for name, thread in threads_to_join:
                if thread is not None:
                    thread.join(timeout=1.0)
                    logging.info("%s monitor thread stopped.", name)

        if vehicle is not None:
            try:
                vehicle.close()
                print("\n🔚 Done — disconnected.")
                logging.info("Vehicle connection closed cleanly.")
            except Exception as e:
                print(f"⚠️ Error closing vehicle: {e}")
                logging.exception("Error closing vehicle: %s", e)


if __name__ == "__main__":
    # Allow running directly from service for backward compatibility
    from main.mission_main import main

    main()
