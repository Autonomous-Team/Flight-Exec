#!/usr/bin/env python3
"""Arm/disarm helpers for ground testing without GPS."""

from __future__ import annotations

import collections
import logging
import time

from dronekit import Vehicle, VehicleMode

from utils.detect_controller import connect_to_first_available, list_candidate_ports
from utils.free_ports import free_acm_usb_ports

if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping


def arm_vehicle(vehicle: Vehicle) -> None:
    """Force-arm motors without GPS by switching to STABILIZE and disabling checks."""
    print("\n[STEP 1] Setting mode to STABILIZE (no GPS needed)...")
    vehicle.mode = VehicleMode("STABILIZE")

    for _ in range(5):
        if vehicle.mode.name == "STABILIZE":
            break
        print("  Waiting for mode change...")
        time.sleep(1)

    print("[STEP 2] Disabling arming checks and arming motors...")
    vehicle.parameters["ARMING_CHECK"] = 0
    vehicle.armed = True

    for _ in range(5):
        if vehicle.armed:
            break
        print("  Waiting for arming...")
        time.sleep(1)

    if vehicle.armed:
        print("✅ Motors armed successfully!")
    else:
        raise RuntimeError("❌ Failed to arm motors.")


def disarm_vehicle(vehicle: Vehicle) -> None:
    """Disarm motors and wait for confirmation."""
    print("\n[STEP 3] Disarming motors...")
    vehicle.armed = False

    for _ in range(5):
        if not vehicle.armed:
            break
        print("  Waiting for disarm...")
        time.sleep(1)

    if not vehicle.armed:
        print("✅ Motors disarmed successfully!")
    else:
        raise RuntimeError("❌ Failed to disarm motors.")


def execute_arm_disarm(hold_seconds: float = 5.0) -> None:
    """Connect to the vehicle, arm, optionally hold, and disarm."""
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)
    vehicle: Vehicle | None = None
    try:
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)
        vehicle, port, baud = connect_to_first_available(candidate_ports)
        print(f"📡 Connected on {port} @ {baud}")

        arm_vehicle(vehicle)
        if hold_seconds > 0:
            print(f"\n⏱ Holding armed state for {hold_seconds:.0f} seconds...")
            time.sleep(hold_seconds)
        disarm_vehicle(vehicle)
    finally:
        if vehicle is not None:
            vehicle.close()
        print("\n🔚 Done — disconnected from flight controller.")


def main() -> None:
    """CLI entry point for running the arm/disarm routine."""
    execute_arm_disarm()


if __name__ == "__main__":
    main()

