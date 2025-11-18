#!/usr/bin/env python3
"""Safety management service for flight operations."""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from dronekit import LocationGlobalRelative, Vehicle, VehicleMode

from utils.geometry_utils import haversine_m

# Global safe landing trigger (thread-safe)
_safe_landing_triggered = threading.Event()
_safe_landing_reason = threading.Lock()
_safe_landing_reason_text = ""

# Global home coordinates (set after takeoff)
HOME_LAT: Optional[float] = None
HOME_LON: Optional[float] = None


def trigger_safe_landing(reason: str) -> None:
    """
    Thread-safe function to trigger safe landing from any monitor thread.
    
    Args:
        reason: Human-readable reason for safe landing trigger
    """
    global _safe_landing_reason_text
    with _safe_landing_reason:
        if not _safe_landing_triggered.is_set():
            _safe_landing_reason_text = reason
            _safe_landing_triggered.set()
            logging.critical("🚨 SAFE LANDING TRIGGERED: %s", reason)
            print(f"\n🚨 SAFE LANDING TRIGGERED: {reason}")


def is_safe_landing_triggered() -> bool:
    """Check if safe landing has been triggered."""
    return _safe_landing_triggered.is_set()


def get_safe_landing_reason() -> str:
    """Get the reason for safe landing trigger."""
    with _safe_landing_reason:
        return _safe_landing_reason_text


def set_home_position(lat: float, lon: float) -> None:
    """
    Set the home position coordinates.
    
    Args:
        lat: Home latitude in degrees
        lon: Home longitude in degrees
    """
    global HOME_LAT, HOME_LON
    HOME_LAT = lat
    HOME_LON = lon
    logging.info("Home position set: lat=%s lon=%s", lat, lon)


def get_home_position() -> tuple[Optional[float], Optional[float]]:
    """
    Get the current home position coordinates.
    
    Returns:
        Tuple of (lat, lon) or (None, None) if not set
    """
    return (HOME_LAT, HOME_LON)


def safe_landing(vehicle: Vehicle, reason: str = "Safety trigger") -> bool:
    """
    Comprehensive safe landing procedure with multiple fallback methods.
    
    Args:
        vehicle: Connected vehicle instance
        reason: Reason for safe landing
    
    Returns:
        True if landing was successful, False otherwise
    """
    print(f"\n🚨 [SAFE LANDING] Initiating safe landing: {reason}")
    logging.critical("SAFE LANDING: Initiating safe landing - %s", reason)

    if vehicle is None:
        logging.error("SAFE LANDING: Vehicle is None, cannot land")
        return False

    try:
        # Check connection first
        if not getattr(vehicle, "connected", False):
            logging.warning("SAFE LANDING: Vehicle not connected, attempting RTL via mode change")
            try:
                vehicle.mode = VehicleMode("RTL")
                logging.info("SAFE LANDING: Set RTL mode (vehicle may reconnect)")
                return True
            except Exception as e:
                logging.error("SAFE LANDING: Could not set RTL mode: %s", e)
                return False

        # Method 1: Try RTL first (most reliable if HOME is set)
        try:
            print("  → Attempting RTL (Return to Launch)...")
            logging.info("SAFE LANDING: Attempting RTL mode")
            vehicle.mode = VehicleMode("RTL")
            time.sleep(2)  # Wait for mode change

            # Verify mode changed
            if getattr(vehicle.mode, "name", "") == "RTL":
                print("  ✅ RTL mode engaged")
                logging.info("SAFE LANDING: RTL mode engaged successfully")
                # Monitor RTL progress
                for i in range(60):  # Wait up to 60 seconds
                    if not getattr(vehicle, "connected", False):
                        logging.warning("SAFE LANDING: Disconnected during RTL")
                        break
                    alt = getattr(vehicle.location.global_relative_frame, "alt", 0.0) or 0.0
                    if alt <= 0.5:
                        print("  ✅ Vehicle landed via RTL")
                        logging.info("SAFE LANDING: Vehicle landed via RTL")
                        return True
                    time.sleep(1)
                return True  # RTL mode set, let it complete
        except Exception as e:
            logging.warning("SAFE LANDING: RTL failed: %s, trying LAND mode", e)

        # Method 2: Try LAND mode
        try:
            print("  → Attempting LAND mode...")
            logging.info("SAFE LANDING: Attempting LAND mode")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(2)

            if getattr(vehicle.mode, "name", "") == "LAND":
                print("  ✅ LAND mode engaged")
                logging.info("SAFE LANDING: LAND mode engaged successfully")
                return True
        except Exception as e:
            logging.warning("SAFE LANDING: LAND mode failed: %s, trying simple_goto", e)

        # Method 3: Try simple_goto to home (if known) then LAND
        home_lat, home_lon = get_home_position()
        if home_lat is not None and home_lon is not None:
            try:
                print("  → Attempting to return to home position...")
                logging.info("SAFE LANDING: Attempting simple_goto to home")
                vehicle.simple_goto(LocationGlobalRelative(home_lat, home_lon, 2.0))
                time.sleep(5)  # Give it time to start moving

                # Then try LAND
                vehicle.mode = VehicleMode("LAND")
                logging.info("SAFE LANDING: Set LAND mode after simple_goto")
                return True
            except Exception as e:
                logging.warning("SAFE LANDING: simple_goto + LAND failed: %s", e)

        # Method 4: Last resort - try to disarm (only if very low altitude)
        try:
            alt = getattr(vehicle.location.global_relative_frame, "alt", 999.0) or 999.0
            if alt < 1.0:  # Only if very close to ground
                print("  → Very low altitude, attempting disarm...")
                logging.warning("SAFE LANDING: Very low altitude (%.2fm), attempting disarm", alt)
                vehicle.armed = False
                return True
        except Exception as e:
            logging.error("SAFE LANDING: Disarm attempt failed: %s", e)

        logging.error("SAFE LANDING: All landing methods failed")
        return False

    except Exception as e:
        logging.exception("SAFE LANDING: Exception during safe landing: %s", e)
        return False

