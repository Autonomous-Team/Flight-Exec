#!/usr/bin/env python3
"""Emergency landing utilities."""

from __future__ import annotations

import logging
import time

from dronekit import LocationGlobalRelative, Vehicle, VehicleMode


def emergency_slow_land(vehicle: Vehicle, logger: logging.Logger | None = None) -> None:
    """Perform emergency slow descent and land."""
    
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.critical("!!! EMERGENCY: Slow descent + land !!!")

    # ---------------------------------------------------------
    # SAFE GUIDED-MODE CHECK (your requested enhancement)
    # ---------------------------------------------------------
    guided_ok = False
    try:
        logger.info("Requesting GUIDED mode...")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(0.5)

        for _ in range(6):  # wait up to 3 seconds
            if vehicle.mode.name == "GUIDED":
                guided_ok = True
                break
            time.sleep(0.5)

        if not guided_ok:
            logger.warning("❌ GUIDED mode not accepted by autopilot.")
        else:
            logger.info("✅ GUIDED mode accepted.")

    except Exception as e:
        logger.warning(f"❌ Could not force GUIDED in emergency: {e}")
        guided_ok = False

    if not guided_ok:
        logger.critical("⚠️ GUIDED unavailable → Emergency LAND initiated.")
        vehicle.mode = VehicleMode("LAND")
        return
    # ---------------------------------------------------------
    # END MODIFICATION
    # ---------------------------------------------------------

    DESCENT_RATE = 0.5
    MIN_ALT = 0.5

    while True:
        loc = vehicle.location.global_relative_frame
        alt = loc.alt

        if alt <= MIN_ALT:
            break

        next_alt = max(MIN_ALT, alt - DESCENT_RATE)
        logger.info(f"Descending: {alt:.2f} → {next_alt:.2f}")

        vehicle.simple_goto(LocationGlobalRelative(loc.lat, loc.lon, next_alt))
        time.sleep(1)

    logger.info("Final LAND command issued.")
    vehicle.mode = VehicleMode("LAND")

    # Wait for landing
    while vehicle.armed:
        loc = vehicle.location.global_relative_frame
        logger.info(f"Landing... alt={loc.alt:.2f}m")
        time.sleep(1)

    logger.info("Landed successfully.")

    # Disarm the vehicle
    logger.info("Disarming vehicle...")
    vehicle.armed = False
    
    for _ in range(10):
        if not vehicle.armed:
            break
        logger.info("Waiting for disarm...")
        time.sleep(1)
    
    if not vehicle.armed:
        logger.info("✅ Vehicle disarmed successfully")
    else:
        logger.warning("⚠️ Vehicle may still be armed")
    
    logger.info("Disconnecting from vehicle...")
    try:
        vehicle.close()
        logger.info("✅ Disconnected from vehicle")
    except Exception as e:
        logger.error(f"Error disconnecting: {e}")
    
    logger.critical("Emergency landing complete. Exiting execution.")
 
