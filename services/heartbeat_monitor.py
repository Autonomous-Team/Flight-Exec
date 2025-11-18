#!/usr/bin/env python3
"""Heartbeat monitoring service for flight operations."""

from __future__ import annotations

import logging
import threading
import time

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle

from services.safety_manager import is_safe_landing_triggered, trigger_safe_landing

def start_heartbeat_monitor(
    vehicle: Vehicle,
    stop_event: threading.Event,
    *,
    warn_seconds: float,
    critical_seconds: float,
    logger_name: str = "HeartbeatMonitor",
) -> threading.Thread:
    """
    Start a background thread that periodically prints heartbeat/connectivity status.
    
    Triggers safe landing on connection loss.
    
    Args:
        vehicle: Connected vehicle instance
        stop_event: Event to signal thread to stop
    
    Returns:
        The Thread object
    """
    def monitor():
        logger = logging.getLogger(logger_name)
        consecutive_critical = 0
        while not stop_event.is_set() and not is_safe_landing_triggered():
            try:
                connected = getattr(vehicle, "connected", False)
                last_hb = getattr(vehicle, "last_heartbeat", None)
                now = time.time()

                if not connected:
                    consecutive_critical += 1
                    if consecutive_critical >= 3:  # 3 consecutive checks ~= 3*warn_seconds
                        trigger_safe_landing("Vehicle disconnected - no connection")
                    else:
                        logger.warning("Vehicle disconnected (check %d/3)", consecutive_critical)
                elif last_hb is None:
                    print("ℹ️ No heartbeat yet.")
                    logger.debug("No heartbeat yet.")
                    consecutive_critical = 0
                else:
                    age = now - last_hb
                    if age > critical_seconds:
                        consecutive_critical += 1
                        print(f"⚠️ Heartbeat critical: last heartbeat {age:.1f}s ago")
                        logger.critical("Heartbeat critical: last heartbeat %.1f s ago", age)
                        if consecutive_critical >= 2:  # 2 consecutive critical = connection lost
                            trigger_safe_landing(f"Heartbeat lost - last heartbeat {age:.1f}s ago")
                    elif age > warn_seconds:
                        print(f"⚠️ Heartbeat delayed: last heartbeat {age:.1f}s ago")
                        logger.warning("Heartbeat delayed: last heartbeat %.1f s ago", age)
                        consecutive_critical = 0
                    else:
                        print(f"ℹ️ Heartbeat OK: {age:.1f}s since last heartbeat")
                        logger.debug("Heartbeat age: %.1f s", age)
                        consecutive_critical = 0

                print(f"ℹ️ vehicle.connected = {connected}")
                logger.debug("vehicle.connected = %s", connected)
            except Exception as e:
                print("⚠️ Heartbeat monitor error:", e)
                logger.exception("Heartbeat monitor error: %s", e)
                consecutive_critical += 1
                if consecutive_critical >= 3:
                    trigger_safe_landing(f"Heartbeat monitor error: {e}")
            # print every HEARTBEAT_WARN_SEC to avoid too-verbose logs
            stop_event.wait(warn_seconds)

    thr = threading.Thread(target=monitor, name="HBMonitor", daemon=True)
    thr.start()
    logging.getLogger(logger_name).info("Heartbeat monitor thread started.")
    return thr

