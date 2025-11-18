#!/usr/bin/env python3
"""Enhanced takeoff and landing service with safety features."""

from __future__ import annotations

import logging
import time
from typing import Optional

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import LocationGlobalRelative, Vehicle, VehicleMode

from services.safety_manager import (
    get_home_position,
    get_safe_landing_reason,
    is_safe_landing_triggered,
    set_home_position,
    trigger_safe_landing,
)

def arm_and_takeoff(
    vehicle: Vehicle,
    target_altitude: float,
    *,
    home_wait_timeout: Optional[int] = None,
) -> None:
    """
    Arm vehicle and take off to target_altitude, saving HOME when valid.
    
    Args:
        vehicle: Connected vehicle instance
        target_altitude: Target altitude in meters
    
    Raises:
        RuntimeError: If arming fails or safe landing is triggered
    """
    print("\n[STEP 1] Checking pre-arm conditions...")
    logging.info("STEP 1: Checking pre-arm conditions.")
    
    for i in range(15):
        if getattr(vehicle, "is_armable", False):
            logging.debug("Vehicle is armable at iteration %s", i)
            break
        print(f"  Waiting for vehicle to initialise (EKF/GPS)... ({i+1}/15)")
        logging.info("Waiting for vehicle to initialise (EKF/GPS)... (%s/15)", i + 1)
        time.sleep(1)
    else:
        print("⚠️ Vehicle not armable after wait; continuing but arming may fail.")
        logging.warning("Vehicle not armable after wait; continuing but arming may fail.")

    print("[STEP 2] Switching to GUIDED mode...")
    logging.info("STEP 2: Switching to GUIDED mode.")
    try:
        vehicle.mode = VehicleMode("GUIDED")
        logging.debug("Set vehicle.mode to GUIDED")
    except Exception as e:
        print("⚠️ Could not set GUIDED mode:", e)
        logging.exception("Could not set GUIDED mode: %s", e)

    for _ in range(5):
        if getattr(vehicle.mode, "name", "") == "GUIDED":
            logging.debug("Vehicle mode is GUIDED")
            break
        print("  Waiting for GUIDED mode...")
        logging.debug("Waiting for GUIDED mode...")
        time.sleep(1)

    print("[STEP 3] Arming motors...")
    logging.info("STEP 3: Arming motors.")
    try:
        vehicle.armed = True
        logging.debug("Issued arm command")
    except Exception as e:
        print("⚠️ Error while arming:", e)
        logging.exception("Error while arming: %s", e)

    for j in range(8):
        if getattr(vehicle, "armed", False):
            logging.debug("Vehicle armed at iteration %s", j)
            break
        print("  Waiting for arming...")
        logging.debug("Waiting for arming... (%s/8)", j + 1)
        time.sleep(1)

    if not getattr(vehicle, "armed", False):
        logging.error("Failed to arm vehicle after waiting.")
        raise RuntimeError("❌ Failed to arm vehicle.")

    print(f"[STEP 4] Taking off to {target_altitude:.1f} meters...")
    logging.info("STEP 4: Taking off to %s meters", target_altitude)
    try:
        vehicle.simple_takeoff(target_altitude)
        logging.debug("Called simple_takeoff(%s)", target_altitude)
    except Exception as e:
        print("⚠️ simple_takeoff command failed:", e)
        logging.exception("simple_takeoff command failed: %s", e)

    if home_wait_timeout is None:
        from config import get_flight_config

        home_wait_timeout = get_flight_config().home_wait_timeout

    # Wait for global_relative_frame to become available (so we can save HOME)
    waited = 0
    while waited < home_wait_timeout:
        lat = getattr(vehicle.location.global_relative_frame, "lat", None)
        lon = getattr(vehicle.location.global_relative_frame, "lon", None)
        logging.debug("Takeoff wait loop iteration %s: lat=%s lon=%s", waited, lat, lon)
        
        if lat is not None and lon is not None:
            set_home_position(float(lat), float(lon))
            print(f"🏠 Saved home position: LAT={lat}, LON={lon}")
            logging.info("Saved HOME position lat=%s lon=%s", lat, lon)
            break
        
        print("  Waiting for valid global_relative_frame (GPS/home)...")
        logging.info("Waiting for valid global_relative_frame (GPS/home)... iteration %s", waited)
        time.sleep(1)
        waited += 1

    if get_home_position()[0] is None:
        print("⚠️ Could not read home location; HOME not saved. Landing/RTL may not return exactly to start.")
        logging.warning("Could not read home location; HOME not saved.")

    # Wait until altitude reached (use small timeout but robust)
    while True:
        # Check for safe landing triggers during takeoff
        if is_safe_landing_triggered():
            reason = get_safe_landing_reason()
            logging.critical("Safe landing triggered during takeoff: %s", reason)
            raise RuntimeError(f"Safe landing triggered: {reason}")

        if not getattr(vehicle, "connected", True):
            logging.error("Vehicle disconnected during takeoff.")
            trigger_safe_landing("Vehicle disconnected during takeoff")
            raise RuntimeError("❌ Vehicle disconnected during takeoff.")

        alt = getattr(vehicle.location.global_relative_frame, "alt", 0.0) or 0.0
        print(f"  → Altitude: {alt:.2f} m")
        logging.debug("Altitude during takeoff: %s m", alt)
        
        if alt >= target_altitude * 0.95:
            print("✅ Target altitude reached")
            logging.info("Target altitude reached: %s m", alt)
            break
        
        time.sleep(1)


def land_and_disarm(vehicle: Vehicle) -> None:
    """
    Command vehicle to land at HOME (if known) and wait for disarm.
    
    Args:
        vehicle: Connected vehicle instance
    """
    from services.safety_manager import get_home_position, safe_landing

    print("\n[STEP 5] Landing back at home...")
    logging.info("STEP 5: Landing back at home.")
    
    try:
        home_lat, home_lon = get_home_position()
        if home_lat is not None and home_lon is not None:
            try:
                vehicle.simple_goto(LocationGlobalRelative(home_lat, home_lon, 2.0))
                print("  → Commanding vehicle to approach home before landing.")
                logging.info("Commanded approach to home (pre-land) lat=%s lon=%s", home_lat, home_lon)
                time.sleep(3)
            except Exception as e:
                print("  ⚠️ simple_goto to home (pre-land) failed:", e)
                logging.exception("simple_goto to home (pre-land) failed: %s", e)
        
        print("  → Switching to LAND mode.")
        logging.info("Switching to LAND mode.")
        try:
            vehicle.mode = VehicleMode("LAND")
            logging.debug("Set vehicle.mode to LAND")
        except Exception as e:
            print("  ⚠️ Could not set LAND mode:", e)
            logging.exception("Could not set LAND mode: %s", e)
            # Fallback to safe landing
            safe_landing(vehicle, "LAND mode failed, using safe landing")

        # Wait for landing (with timeout)
        start = time.time()
        timeout = 120  # seconds
        logging.debug("Starting landing wait loop with timeout %s seconds", timeout)
        
        while True:
            if not getattr(vehicle, "connected", True):
                print("⚠️ Vehicle disconnected during landing.")
                logging.error("Vehicle disconnected during landing loop.")
                break

            alt = getattr(vehicle.location.global_relative_frame, "alt", 0.0) or 0.0
            print(f"  → Altitude: {alt:.2f} m")
            logging.debug("Landing loop altitude: %s m", alt)
            
            if alt <= 0.2:
                print("✅ Landed successfully")
                logging.info("Landed successfully at altitude: %s", alt)
                break
            
            if time.time() - start > timeout:
                print("⚠️ Landing timeout reached; breaking.")
                logging.warning("Landing timeout reached after %s seconds", timeout)
                break
            
            time.sleep(1)

        print("[STEP 6] Waiting for auto-disarm...")
        logging.info("STEP 6: Waiting for auto-disarm.")
        
        for k in range(10):
            if not getattr(vehicle, "armed", True):
                print("✅ Vehicle disarmed")
                logging.info("Vehicle disarmed after %s checks", k)
                break
            print("  Waiting for disarm...")
            logging.debug("Waiting for disarm... (%s/10)", k + 1)
            time.sleep(1)
        else:
            print("⚠️ Vehicle still armed after wait (auto-disarm didn't happen).")
            logging.warning("Vehicle still armed after wait; auto-disarm didn't happen.")

    except Exception as e:
        print("⚠️ Exception during land_and_disarm:", e)
        logging.exception("Exception during land_and_disarm: %s", e)
        # Try safe landing as last resort
        safe_landing(vehicle, f"Exception during land_and_disarm: {e}")

