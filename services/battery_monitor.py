#!/usr/bin/env python3
"""Battery monitoring service for flight operations."""

from __future__ import annotations

import logging
import threading
import time

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle

from services.safety_manager import is_safe_landing_triggered, trigger_safe_landing

def start_battery_monitor(
    vehicle: Vehicle,
    stop_event: threading.Event,
    *,
    critical_threshold: float,
    low_threshold: float,
    logger_name: str = "BatteryMonitor",
) -> threading.Thread:
    """
    Start a background thread that periodically logs battery status.
    
    Triggers safe landing on critical battery levels.
    
    Args:
        vehicle: Connected vehicle instance
        stop_event: Event to signal thread to stop
    
    Returns:
        The Thread object
    """
    def monitor():
        logger = logging.getLogger(logger_name)
        while not stop_event.is_set() and not is_safe_landing_triggered():
            try:
                if not getattr(vehicle, "connected", False):
                    stop_event.wait(1.0)
                    continue

                battery = getattr(vehicle, "battery", None)
                if battery is None:
                    logging.debug("Battery object not available yet")
                    stop_event.wait(1.0)
                    continue

                voltage = getattr(battery, "voltage", None)
                current = getattr(battery, "current", None)
                level = getattr(battery, "level", None)

                # Log battery data
                if voltage is not None:
                    logger.info("BATTERY: voltage=%.2fV", voltage)
                if current is not None:
                    logger.info("BATTERY: current=%.2fA", current)
                if level is not None:
                    logger.info("BATTERY: level=%.1f%%", level)
                    # Trigger safe landing on critical battery
                    if level < critical_threshold:
                        trigger_safe_landing(f"Critical battery level: {level:.1f}%")
                    elif level < low_threshold:
                        logger.warning("BATTERY: Low battery warning - %.1f%% remaining", level)

                # Check voltage thresholds (if level not available)
                if voltage is not None and level is None:
                    # Typical LiPo: 3.0V per cell is critical, 3.3V is low
                    # Assuming 4S battery (12V critical, 13.2V low)
                    if voltage < 12.0:
                        trigger_safe_landing(f"Critical battery voltage: {voltage:.2f}V")
                    elif voltage < 13.2:
                        logger.warning("BATTERY: Low battery voltage warning - %.2fV", voltage)

                # Log all battery attributes for comprehensive logging
                if hasattr(battery, "__dict__"):
                    for attr, value in battery.__dict__.items():
                        if value is not None and not attr.startswith("_"):
                            logger.debug("BATTERY: %s=%s", attr, value)

            except Exception as e:
                logger.exception("Battery monitor error: %s", e)
                # If battery monitor fails repeatedly, trigger safe landing
                trigger_safe_landing(f"Battery monitor error: {e}")

            # Check every 2 seconds (frequent enough for real-time monitoring)
            stop_event.wait(2.0)

    thr = threading.Thread(target=monitor, name="BatteryMonitor", daemon=True)
    thr.start()
    logging.getLogger(logger_name).info("Battery monitor thread started.")
    return thr

