#!/usr/bin/env python3
"""Position hold service for maintaining position during flight."""

from __future__ import annotations

import logging
import time
from typing import Optional

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import LocationGlobalRelative, Vehicle

from services.safety_manager import (
    get_home_position,
    get_safe_landing_reason,
    is_safe_landing_triggered,
    trigger_safe_landing,
)
from utils.geometry_utils import haversine_m

def hold_position(
    vehicle: Vehicle,
    hold_time: Optional[int] = None,
    target_alt: Optional[float] = None,
    *,
    goto_threshold_m: Optional[float] = None,
    hold_rate_hz: Optional[float] = None,
) -> bool:
    """
    Actively hold the HOME position.
    
    Args:
        vehicle: Connected vehicle instance
        hold_time: Time in seconds to hold position
        target_alt: Target altitude in meters
    
    Returns:
        True if hold completed successfully, False if error occurred
    """
    from config import get_flight_config

    cfg = get_flight_config()

    if hold_time is None:
        hold_time = int(cfg.hold_seconds)
    if target_alt is None:
        target_alt = float(cfg.target_altitude)
    if goto_threshold_m is None:
        goto_threshold_m = float(cfg.goto_threshold_m)
    if hold_rate_hz is None:
        hold_rate_hz = float(cfg.hold_rate_hz)

    print("\n[HOLD] Actively holding position...")
    logging.info("HOLD: Actively holding position for %s seconds", hold_time)
    
    home_lat, home_lon = get_home_position()
    if home_lat is None or home_lon is None:
        print("⚠️ HOME not set; hold cannot continue.")
        logging.warning("HOME not set; aborting hold_position.")
        trigger_safe_landing("HOME not set during hold")
        return False

    target = LocationGlobalRelative(home_lat, home_lon, target_alt)
    logging.debug("Hold target set to lat=%s lon=%s alt=%s", home_lat, home_lon, target_alt)

    start = time.time()
    iteration = 0

    try:
        while time.time() - start < float(hold_time):
            # Check for safe landing triggers
            if is_safe_landing_triggered():
                reason = get_safe_landing_reason()
                logging.warning("Safe landing triggered during hold: %s", reason)
                return False

            iteration += 1
            logging.debug("Hold loop iteration %s", iteration)

            # Connection check
            if not getattr(vehicle, "connected", True):
                print("❌ Vehicle disconnected during hold.")
                logging.error("Vehicle disconnected during hold at iteration %s", iteration)
                trigger_safe_landing("Vehicle disconnected during hold")
                return False

            # Current position check
            loc = vehicle.location.global_relative_frame
            lat = getattr(loc, "lat", None)
            lon = getattr(loc, "lon", None)
            logging.debug("Hold loop loc lat=%s lon=%s", lat, lon)

            if loc is None or lat is None or lon is None:
                print("⚠️ No valid GPS fix; cannot hold.")
                logging.warning("No valid GPS fix during hold at iteration %s", iteration)
                trigger_safe_landing("No valid GPS fix during hold")
                return False

            dist = haversine_m(lat, lon, home_lat, home_lon)
            print(f"  → Distance to HOME: {dist:.2f} m")
            logging.debug("Distance to HOME at iteration %s: %s m", iteration, dist)

            # If far from target → send goto
            if dist > goto_threshold_m:
                try:
                    vehicle.simple_goto(target)
                    print("  → Sent simple_goto to HOME")
                    logging.info("Sent simple_goto to HOME at iteration %s (dist=%s m)", iteration, dist)
                except Exception as goto_err:
                    print(f"❌ simple_goto failed: {goto_err}")
                    logging.exception("simple_goto failed at iteration %s: %s", iteration, goto_err)
                    trigger_safe_landing(f"simple_goto failed during hold: {goto_err}")
                    return False
            else:
                print("  → Within threshold; holding")
                logging.debug("Within threshold; not re-sending goto at iteration %s", iteration)

            time.sleep(1.0 / hold_rate_hz)

        logging.info("Hold completed normally after %s iterations", iteration)
        return True  # Hold completed normally

    except Exception as e:
        print(f"❌ Exception inside hold_position(): {e}")
        logging.exception("Exception inside hold_position at iteration %s: %s", iteration, e)
        trigger_safe_landing(f"Exception in hold_position: {e}")
        return False

