#!/usr/bin/env python3
"""High-level takeoff and landing operations."""

from __future__ import annotations

import collections
import logging
import time
from typing import Protocol

from dronekit import Vehicle, VehicleMode

from utils.detect_controller import connect_to_first_available, list_candidate_ports
from utils.free_ports import free_acm_usb_ports

if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping


class SupportsVehicle(Protocol):
    """Minimal protocol to avoid tightly coupling to dronekit.Vehicle in tests."""

    is_armable: bool
    mode: VehicleMode
    armed: bool

    def simple_takeoff(self, target_altitude: float) -> None: ...

    class location:  # type: ignore
        class global_relative_frame:  # type: ignore
            alt: float


def arm_and_takeoff(vehicle: Vehicle, target_altitude: float) -> None:
    """Arm the vehicle and ascend to the requested altitude."""
    print("\n[STEP 1] Checking pre-arm conditions...")
    for _ in range(15):  # Wait up to 15 seconds for armable state
        if vehicle.is_armable:
            break
        print("  Waiting for vehicle to initialise (EKF/GPS)...")
        time.sleep(1)

    print("[STEP 2] Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    for _ in range(5):
        if vehicle.mode.name == "GUIDED":
            break
        print("  Waiting for GUIDED mode...")
        time.sleep(1)

    print("[STEP 3] Arming motors...")
    vehicle.armed = True
    for _ in range(5):
        if vehicle.armed:
            break
        print("  Waiting for arming...")
        time.sleep(1)
    print("✅ Vehicle armed")

    print(f"[STEP 4] Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  → Altitude: {alt:.2f} m")
        if alt >= target_altitude * 0.95:
            print("✅ Target altitude reached")
            break
        time.sleep(1)


def land_and_disarm(vehicle: Vehicle) -> None:
    """Initiate landing and wait for disarm."""
    print("\n[STEP 5] Landing initiated...")
    vehicle.mode = VehicleMode("LAND")

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  → Altitude: {alt:.2f} m")
        if alt <= 0.2:
            print("✅ Landed successfully")
            break
        time.sleep(1)

    print("[STEP 6] Waiting for auto-disarm...")
    for _ in range(10):
        if not vehicle.armed:
            break
        print("  Waiting for disarm...")
        time.sleep(1)
    print("✅ Vehicle disarmed")


def execute_takeoff_land(target_altitude: float = 5.0, hover_seconds: float = 10.0) -> None:
    """Connect, take off, hover, and land."""
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)
    vehicle: Vehicle | None = None
    try:
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)
        vehicle, port, baud = connect_to_first_available(candidate_ports)
        print(f"📡 Connected on {port} @ {baud}")

        arm_and_takeoff(vehicle, target_altitude)
        if hover_seconds > 0:
            print(f"\n[HOVER] Holding position for {hover_seconds:.0f} seconds...")
            time.sleep(hover_seconds)
        land_and_disarm(vehicle)
    finally:
        if vehicle is not None:
            vehicle.close()
        print("\n🔚 Done — disconnected from flight controller.")


def main() -> None:
    """CLI entry point for the takeoff/land routine."""
    execute_takeoff_land()


if __name__ == "__main__":
    main()

