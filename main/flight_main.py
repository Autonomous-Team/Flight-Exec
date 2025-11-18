#!/usr/bin/env python3
"""
Main flight execution script with comprehensive safety monitoring.

This script orchestrates all flight services including:
- Connection management
- Safety monitoring (battery, heartbeat, EKF, etc.)
- System monitoring (Jetson stats)
- Flight operations (takeoff, hold, land)
- Safe landing procedures
"""

from __future__ import annotations

import collections
import logging
import threading
from typing import Optional

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle

from services.battery_monitor import start_battery_monitor
from services.enhanced_takeoff_service import arm_and_takeoff, land_and_disarm
from services.fc_telemetry_monitor import start_fc_telemetry_monitor
from services.heartbeat_monitor import start_heartbeat_monitor
from services.jetson_monitor import start_jetson_monitor
from services.position_hold_service import hold_position
from services.safety_manager import (
    get_safe_landing_reason,
    is_safe_landing_triggered,
    safe_landing,
)
from utils.detect_controller import detect_fc_and_prepare
from utils.logging_utils import setup_logging
from utils.port_utils import report_port_users

# Fix for older code expecting MutableMapping at top-level
if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore

def main() -> None:
    """Main flight execution function."""
    from config import get_flight_config

    cfg = get_flight_config()

    # Setup logging with config
    setup_logging(
        detailed=cfg.logging_detailed,
        log_dir=cfg.logging_dir,
        log_prefix="flight",
    )

    vehicle: Optional[Vehicle] = None
    hb_stop = threading.Event()
    hb_thread: Optional[threading.Thread] = None
    battery_thread: Optional[threading.Thread] = None
    jetson_thread: Optional[threading.Thread] = None
    fc_telemetry_thread: Optional[threading.Thread] = None

    try:
        # 1) Report processes using serial ports (no killing)
        report_port_users()

        # 2) Detect flight controller and connect
        vehicle, port, baud = detect_fc_and_prepare()
        logging.info("Connected vehicle object: %s, port=%s, baud=%s", vehicle, port, baud)

        # 3) Start all monitoring threads based on config
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

        # 4) Arm and takeoff
        try:
            arm_and_takeoff(
                vehicle,
                cfg.target_altitude,
                home_wait_timeout=cfg.home_wait_timeout,
            )
            # Check for safe landing trigger after takeoff
            if is_safe_landing_triggered():
                reason = get_safe_landing_reason()
                safe_landing(vehicle, reason)
                return
        except Exception as e:
            logging.exception("Takeoff failed: %s", e)
            safe_landing(vehicle, f"Takeoff failed: {e}")
            return

        # 5) Active hold with safety wrapper and safe landing monitoring
        try:
            hold_ok = hold_position(
                vehicle,
                hold_time=int(cfg.hold_seconds),
                target_alt=cfg.target_altitude,
                goto_threshold_m=cfg.goto_threshold_m,
                hold_rate_hz=cfg.hold_rate_hz,
            )

            # Check for safe landing triggers during/after hold
            if is_safe_landing_triggered():
                reason = get_safe_landing_reason()
                print(f"\n🚨 Safe landing triggered: {reason}")
                safe_landing(vehicle, reason)
                return

            if not hold_ok:
                print("\n⚠️ Hold-position failed — initiating SAFE LANDING before any disconnect...")
                logging.warning("Hold-position failed; initiating safe landing.")
                safe_landing(vehicle, "Hold position failed")
                return  # Prevent executing remaining flight logic
        except Exception as e:
            logging.exception("Hold position exception: %s", e)
            safe_landing(vehicle, f"Hold position exception: {e}")
            return

        # Check for safe landing triggers before normal landing
        if is_safe_landing_triggered():
            reason = get_safe_landing_reason()
            print(f"\n🚨 Safe landing triggered: {reason}")
            safe_landing(vehicle, reason)
            return

        # 6) Land and disarm (normal path)
        try:
            land_and_disarm(vehicle)
        except Exception as e:
            logging.exception("Normal landing failed: %s", e)
            safe_landing(vehicle, f"Normal landing failed: {e}")

    except KeyboardInterrupt:
        print("\n✋ KeyboardInterrupt received. Initiating safe landing.")
        logging.warning("KeyboardInterrupt received. Initiating safe landing.")
        if vehicle is not None:
            safe_landing(vehicle, "KeyboardInterrupt - user requested")
    except Exception as exc:
        print("\n❌ Unhandled error:", exc)
        logging.exception("Unhandled error in main(): %s", exc)
        # Attempt safe landing on any exception
        if vehicle is not None:
            safe_landing(vehicle, f"Unhandled exception: {exc}")
    finally:
        # Stop all monitoring threads
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
                print("\n🔚 Done — disconnected from flight controller.")
                logging.info("Vehicle connection closed cleanly.")
            except Exception as e:
                print("⚠️ Error closing vehicle:", e)
                logging.exception("Error closing vehicle: %s", e)
        else:
            print("\n🔚 Done — no vehicle to disconnect.")
            logging.info("Main finished with no vehicle to disconnect.")


if __name__ == "__main__":
    main()

