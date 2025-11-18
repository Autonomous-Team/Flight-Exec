#!/usr/bin/env python3
"""Heartbeat monitoring service for flight operations."""

from __future__ import annotations

import logging
import threading
import time

from dronekit import Vehicle

from services.safety_manager import is_safe_landing_triggered, trigger_safe_landing

# Heartbeat thresholds
HEARTBEAT_WARN_SEC = 5  # if last_heartbeat older than this, warn
HEARTBEAT_CRITICAL_SEC = 15  # if older than this, consider connection lost


def start_heartbeat_monitor(vehicle: Vehicle, stop_event: threading.Event) -> threading.Thread:
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
        consecutive_critical = 0
        while not stop_event.is_set() and not is_safe_landing_triggered():
            try:
                connected = getattr(vehicle, "connected", False)
                last_hb = getattr(vehicle, "last_heartbeat", None)
                now = time.time()

                if not connected:
                    consecutive_critical += 1
                    if consecutive_critical >= 3:  # 3 consecutive checks = ~15 seconds
                        trigger_safe_landing("Vehicle disconnected - no connection")
                    else:
                        logging.warning("Vehicle disconnected (check %d/3)", consecutive_critical)
                elif last_hb is None:
                    print("ℹ️ No heartbeat yet.")
                    logging.debug("No heartbeat yet.")
                    consecutive_critical = 0
                else:
                    age = now - last_hb
                    if age > HEARTBEAT_CRITICAL_SEC:
                        consecutive_critical += 1
                        print(f"⚠️ Heartbeat critical: last heartbeat {age:.1f}s ago")
                        logging.critical("Heartbeat critical: last heartbeat %.1f s ago", age)
                        if consecutive_critical >= 2:  # 2 consecutive critical = connection lost
                            trigger_safe_landing(f"Heartbeat lost - last heartbeat {age:.1f}s ago")
                    elif age > HEARTBEAT_WARN_SEC:
                        print(f"⚠️ Heartbeat delayed: last heartbeat {age:.1f}s ago")
                        logging.warning("Heartbeat delayed: last heartbeat %.1f s ago", age)
                        consecutive_critical = 0
                    else:
                        print(f"ℹ️ Heartbeat OK: {age:.1f}s since last heartbeat")
                        logging.debug("Heartbeat age: %.1f s", age)
                        consecutive_critical = 0

                print(f"ℹ️ vehicle.connected = {connected}")
                logging.debug("vehicle.connected = %s", connected)
            except Exception as e:
                print("⚠️ Heartbeat monitor error:", e)
                logging.exception("Heartbeat monitor error: %s", e)
                consecutive_critical += 1
                if consecutive_critical >= 3:
                    trigger_safe_landing(f"Heartbeat monitor error: {e}")
            # print every HEARTBEAT_WARN_SEC to avoid too-verbose logs
            stop_event.wait(HEARTBEAT_WARN_SEC)

    thr = threading.Thread(target=monitor, name="HBMonitor", daemon=True)
    thr.start()
    logging.info("Heartbeat monitor thread started.")
    return thr

