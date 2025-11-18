#!/usr/bin/env python3
"""Safe disconnect utilities for drone operations."""

from __future__ import annotations

import logging
import time

from dronekit import Vehicle


def safe_disconnect(
    vehicle: Vehicle,
    logger: logging.Logger | None = None,
    max_ground_alt: float = 0.5,
    max_wait_time: int = 30,
) -> None:
    """Disconnect from vehicle only if safe conditions are met.
    
    Checks that the drone is on the ground (altitude <= max_ground_alt) and
    disarmed before disconnecting. If conditions are not met within max_wait_time,
    keeps connection open for safety.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        logger: Optional logger instance. If None, uses default logger.
        max_ground_alt: Maximum altitude considered "on ground" in meters (default: 0.5)
        max_wait_time: Maximum seconds to wait for safe conditions (default: 30)
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info("Checking safe disconnect conditions...")
    logger.info(f"Waiting for: 1) Drone on ground (alt <= {max_ground_alt}m), 2) Drone disarmed")
    
    wait_count = 0
    while wait_count < max_wait_time:
        loc = vehicle.location.global_relative_frame
        alt = loc.alt
        armed = vehicle.armed
        
        logger.debug(f"Safety check: alt={alt:.2f}m, armed={armed}")
        
        if alt <= max_ground_alt and not armed:
            logger.info("✅ Safe conditions met: Drone on ground and disarmed")
            try:
                vehicle.close()
                logger.info("✅ Disconnected from vehicle safely")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
            return
        
        wait_count += 1
        if wait_count % 5 == 0:
            logger.warning(
                f"Still waiting for safe conditions... "
                f"(alt={alt:.2f}m, armed={armed}, {max_wait_time - wait_count}s remaining)"
            )
        time.sleep(1)
    
    logger.warning(
        f"⚠️ Timeout waiting for safe conditions. "
        f"Current state: alt={vehicle.location.global_relative_frame.alt:.2f}m, "
        f"armed={vehicle.armed}. Keeping connection open for safety."
    )

