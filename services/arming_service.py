#!/usr/bin/env python3
"""Arm/disarm service for ground testing without GPS."""

from __future__ import annotations

import collections
import logging
import threading
import time
from typing import Optional

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle, VehicleMode

from services.safety_manager import (
    get_safe_landing_reason,
    is_safe_landing_triggered,
    safe_landing,
)
from utils.detect_controller import connect_to_first_available, list_candidate_ports
from utils.logging_utils import setup_logging
from utils.port_utils import report_port_users


def arm_vehicle(vehicle: Vehicle, disable_checks: bool = True) -> None:
    """
    Force-arm motors without GPS by switching to STABILIZE and optionally disabling checks.
    
    Args:
        vehicle: Connected vehicle instance
        disable_checks: If True, disable arming checks (for ground testing)
    
    Raises:
        RuntimeError: If arming fails
    """
    print("\n[STEP 1] Setting mode to STABILIZE (no GPS needed)...")
    logging.info("STEP 1: Setting mode to STABILIZE")
    
    try:
        vehicle.mode = VehicleMode("STABILIZE")
        logging.debug("Set vehicle.mode to STABILIZE")
    except Exception as e:
        print(f"⚠️ Could not set STABILIZE mode: {e}")
        logging.exception("Could not set STABILIZE mode: %s", e)

    for i in range(5):
        if getattr(vehicle.mode, "name", "") == "STABILIZE":
            logging.debug("Vehicle mode is STABILIZE at iteration %s", i)
            break
        print("  Waiting for mode change...")
        logging.debug("Waiting for mode change... (%s/5)", i + 1)
        time.sleep(1)

    if disable_checks:
        print("[STEP 2] Disabling arming checks and arming motors...")
        logging.info("STEP 2: Disabling arming checks and arming motors")
        try:
            vehicle.parameters["ARMING_CHECK"] = 0
            logging.debug("Set ARMING_CHECK parameter to 0")
        except Exception as e:
            print(f"⚠️ Could not disable arming checks: {e}")
            logging.warning("Could not disable arming checks: %s", e)
    else:
        print("[STEP 2] Arming motors (with checks enabled)...")
        logging.info("STEP 2: Arming motors (with checks enabled)")

    try:
        vehicle.armed = True
        logging.debug("Issued arm command")
    except Exception as e:
        print(f"⚠️ Error while arming: {e}")
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
        raise RuntimeError("❌ Failed to arm motors.")


def disarm_vehicle(vehicle: Vehicle) -> None:
    """
    Disarm motors and wait for confirmation.
    
    Args:
        vehicle: Connected vehicle instance
    
    Raises:
        RuntimeError: If disarm fails
    """
    print("\n[STEP 3] Disarming motors...")
    logging.info("STEP 3: Disarming motors")
    
    try:
        vehicle.armed = False
        logging.debug("Issued disarm command")
    except Exception as e:
        print(f"⚠️ Error while disarming: {e}")
        logging.exception("Error while disarming: %s", e)

    for k in range(10):
        if not getattr(vehicle, "armed", True):
            logging.debug("Vehicle disarmed at iteration %s", k)
            break
        print("  Waiting for disarm...")
        logging.debug("Waiting for disarm... (%s/10)", k + 1)
        time.sleep(1)

    if getattr(vehicle, "armed", True):
        logging.error("Failed to disarm vehicle after waiting.")
        raise RuntimeError("❌ Failed to disarm motors.")


def execute_arm_disarm(
    hold_seconds: Optional[float] = None,
    disable_checks: Optional[bool] = None,
    enable_safety_monitoring: bool = True,
) -> None:
    """
    Connect to the vehicle, arm, optionally hold, and disarm with safety monitoring.
    
    Args:
        hold_seconds: Time in seconds to hold armed state (0 to skip). If None, uses config.
        disable_checks: If True, disable arming checks for ground testing. If None, uses config.
        enable_safety_monitoring: If True, enable safety monitoring threads
    """
    from config import get_arming_config

    cfg = get_arming_config()
    
    # Use config values if not provided
    if hold_seconds is None:
        hold_seconds = cfg.hold_seconds
    if disable_checks is None:
        disable_checks = cfg.disable_checks

    # Setup logging with config
    setup_logging(detailed=cfg.logging_detailed, log_dir=cfg.logging_dir, log_prefix="arming")
    
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
            from services.battery_monitor import start_battery_monitor
            from services.heartbeat_monitor import start_heartbeat_monitor
            from services.jetson_monitor import start_jetson_monitor
            from services.fc_telemetry_monitor import start_fc_telemetry_monitor

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
            logging.info("Safety monitoring threads started")

        # Arm vehicle
        try:
            arm_vehicle(vehicle, disable_checks=disable_checks)
            print("✅ Motors armed successfully!")
        except Exception as e:
            logging.exception("Arming failed: %s", e)
            raise

        # Hold armed state if requested
        if hold_seconds > 0:
            print(f"\n⏱ Holding armed state for {hold_seconds:.0f} seconds...")
            logging.info("Holding armed state for %s seconds", hold_seconds)
            
            start_time = time.time()
            while time.time() - start_time < hold_seconds:
                # Check for safety triggers
                if enable_safety_monitoring and is_safe_landing_triggered():
                    reason = get_safe_landing_reason()
                    print(f"\n🚨 Safety trigger during hold: {reason}")
                    logging.warning("Safety trigger during hold: %s", reason)
                    # For ground testing, we'll just disarm instead of safe landing
                    break
                
                # Check connection
                if not getattr(vehicle, "connected", False):
                    print("⚠️ Vehicle disconnected during hold")
                    logging.warning("Vehicle disconnected during hold")
                    break
                
                time.sleep(0.5)

        # Disarm vehicle
        try:
            disarm_vehicle(vehicle)
            print("✅ Motors disarmed successfully!")
        except Exception as e:
            logging.exception("Disarming failed: %s", e)
            # Try safe landing as fallback if vehicle is still armed
            if getattr(vehicle, "armed", False):
                print("⚠️ Attempting emergency disarm via safe landing...")
                safe_landing(vehicle, "Disarm failed, using safe landing")

    except KeyboardInterrupt:
        print("\n✋ KeyboardInterrupt received. Disarming vehicle.")
        logging.warning("KeyboardInterrupt received. Disarming vehicle.")
        if vehicle is not None and getattr(vehicle, "armed", False):
            try:
                disarm_vehicle(vehicle)
            except Exception:
                safe_landing(vehicle, "KeyboardInterrupt - emergency disarm")
    except Exception as exc:
        print(f"\n❌ Unhandled error: {exc}")
        logging.exception("Unhandled error in execute_arm_disarm: %s", exc)
        if vehicle is not None and getattr(vehicle, "armed", False):
            try:
                disarm_vehicle(vehicle)
            except Exception:
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
                print("\n🔚 Done — disconnected from flight controller.")
                logging.info("Vehicle connection closed cleanly.")
            except Exception as e:
                print(f"⚠️ Error closing vehicle: {e}")
                logging.exception("Error closing vehicle: %s", e)


if __name__ == "__main__":
    # Allow running directly from service for backward compatibility
    from main.arming_main import main

    main()
