#!/usr/bin/env python3
"""Arm/disarm helpers for ground testing without GPS."""

from __future__ import annotations

import collections

# Fix dronekit compatibility for Python 3.10+
if not hasattr(collections, "MutableMapping"):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

import logging
import time

from dronekit import Vehicle, VehicleMode

from utils.detect_controller import connect_to_first_available, list_candidate_ports
from utils.free_ports import free_acm_usb_ports
from utils.logger import setup_logger
from utils.safe_disconnect import safe_disconnect


def arm_vehicle(vehicle: Vehicle, logger: logging.Logger | None = None) -> None:
    """Force-arm motors without GPS by switching to STABILIZE and disabling checks.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info("[STEP 1] Setting mode to STABILIZE (no GPS needed)...")
    vehicle.mode = VehicleMode("STABILIZE")

    for _ in range(5):
        if vehicle.mode.name == "STABILIZE":
            break
        logger.info("  Waiting for mode change...")
        time.sleep(1)

    logger.info("[STEP 2] Disabling arming checks and arming motors...")
    vehicle.parameters["ARMING_CHECK"] = 0
    vehicle.armed = True

    for _ in range(5):
        if vehicle.armed:
            break
        logger.info("  Waiting for arming...")
        time.sleep(1)

    if vehicle.armed:
        logger.info("✅ Motors armed successfully!")
    else:
        raise RuntimeError("❌ Failed to arm motors.")


def disarm_vehicle(vehicle: Vehicle, logger: logging.Logger | None = None) -> None:
    """Disarm motors and wait for confirmation.
    
    Args:
        vehicle: Connected dronekit vehicle instance
        logger: Optional logger instance
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    logger.info("[STEP 3] Disarming motors...")
    vehicle.armed = False

    for _ in range(5):
        if not vehicle.armed:
            break
        logger.info("  Waiting for disarm...")
        time.sleep(1)

    if not vehicle.armed:
        logger.info("✅ Motors disarmed successfully!")
    else:
        raise RuntimeError("❌ Failed to disarm motors.")


def execute_arm_disarm(hold_seconds: float = 5.0) -> None:
    """Connect to the vehicle, arm, optionally hold, and disarm.
    
    Args:
        hold_seconds: Duration to hold armed state in seconds (default: 5.0)
    """
    # Setup logger
    logger = setup_logger()
    
    # Silence verbose dronekit logs
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)
    
    vehicle: Vehicle | None = None
    try:
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)
        vehicle, port, baud = connect_to_first_available(candidate_ports)
        logger.info(f"📡 Connected on {port} @ {baud}")

        arm_vehicle(vehicle, logger)
        if hold_seconds > 0:
            logger.info(f"⏱ Holding armed state for {hold_seconds:.0f} seconds...")
            time.sleep(hold_seconds)
        disarm_vehicle(vehicle, logger)
    except Exception as e:
        logger.exception(f"ERROR during arm/disarm: {e}")
    finally:
        # Disconnect only if safe conditions are met
        if vehicle:
            safe_disconnect(vehicle, logger)
        else:
            logger.info("🔚 Done — No vehicle connection.")


def main() -> None:
    """CLI entry point for running the arm/disarm routine."""
    execute_arm_disarm()


if __name__ == "__main__":
    main()

