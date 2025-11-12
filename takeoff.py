#!/usr/bin/env python3
import collections

# ---------------- COMPATIBILITY PATCH ----------------
# Fix for Python 3.10+ where MutableMapping moved to collections.abc
if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping

import time
import logging
import subprocess
import serial
import serial.tools.list_ports
from dronekit import connect, VehicleMode

# ---------------- SETUP ----------------
logging.getLogger("dronekit").setLevel(logging.CRITICAL)

PORT = "/dev/ttyACM0"  # Jetson USB port for Pixhawk/Cube
BAUD = 115200  # Standard USB baud rate
TARGET_ALT = 5  # Target altitude in meters


# ---------------- PORT CLEANUP ----------------
def free_acm_usb_ports() -> None:
    """Free busy /dev/ttyACM* and /dev/ttyUSB* ports by killing their processes."""
    print("\n🧹 Checking for busy /dev/ttyACM* and /dev/ttyUSB* ports...")

    try:
        for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*"]:
            result = subprocess.run(
                ["lsof", pattern],
                capture_output=True,
                text=True
            )
            if result.returncode == 0 and result.stdout.strip():
                print(f"⚠️ Ports in use for {pattern}:\n{result.stdout}")
                for line in result.stdout.strip().split("\n")[1:]:
                    parts = line.split()
                    if len(parts) >= 2 and parts[1].isdigit():
                        pid = parts[1]
                        print(f"  🔪 Killing PID {pid} (from {pattern})")
                        subprocess.run(["sudo", "kill", "-9", pid], check=False)
                print(f"✅ Freed all processes using {pattern}")
            else:
                print(f"✅ No busy ports found for {pattern}")
    except FileNotFoundError:
        print("⚠️ 'lsof' not found; skipping port cleanup.")
    except Exception as e:
        print(f"⚠️ Could not check or free ports: {e}")


# ---------------- PERMISSION FIX ----------------
def fix_port_permissions() -> None:
    """Ensure /dev/ttyACM* and /dev/ttyUSB* have correct permissions."""
    print("\n🔐 Setting permissions for /dev/ttyACM* and /dev/ttyUSB*...")
    ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    for port in ports:
        try:
            result = subprocess.run(["sudo", "chmod", "666", port],
                                    check=False, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"✅ Permissions set for {port} (rw for all users)")
            else:
                print(f"⚠️ Could not set permissions for {port} (may not exist)")
        except Exception as e:
            print(f"  ❌ Error setting permissions for {port}: {e}")


# ---------------- FC DETECTION ----------------
def detect_fc_connection():
    """Automatically detect connected flight controller port and working baud rate."""
    print("🔍 Searching for flight controller...")
    possible_ports = [
        p.device for p in serial.tools.list_ports.comports()
        if "ACM" in p.device or "USB" in p.device
    ]
    if not possible_ports:
        raise RuntimeError("❌ No flight controller found on /dev/ttyACM* or /dev/ttyUSB*")

    print(f"Found possible ports: {possible_ports}")
    common_bauds = [115200, 57600, 921600]

    for port in possible_ports:
        for baud in common_bauds:
            try:
                print(f"  → Trying {port} @ {baud}...")
                vehicle = connect(port, baud=baud, wait_ready=True, timeout=10)
                print(f"✅ Connected successfully on {port} @ {baud}")
                return vehicle, port, baud
            except Exception as e:
                print(f"  ✖ {port} @ {baud} failed: {e}")
                continue

    raise RuntimeError("❌ Could not connect to any detected port/baud combination.")


# ---------------- TAKEOFF FUNCTION ----------------
def arm_and_takeoff(vehicle, target_altitude: float) -> None:
    """Arms vehicle and takes off to target altitude."""
    print("\n[STEP 1] Checking pre-arm conditions...")
    while not vehicle.is_armable:
        print("  Waiting for vehicle to initialise (EKF/GPS)...")
        time.sleep(1)

    print("[STEP 2] Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("  Waiting for GUIDED mode...")
        time.sleep(1)

    print("[STEP 3] Preparing to arm motors...")

    # 3-second countdown before arming
    for i in range(3, 0, -1):
        print(f"  ⏳ Arming in {i}...")
        time.sleep(1)

    print("[STEP 4] Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("  Waiting for arming...")
        time.sleep(1)
    print("✅ Vehicle armed")

    print(f"[STEP 5] Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  → Altitude: {alt:.2f} m")
        if alt >= target_altitude * 0.95:
            print("✅ Target altitude reached")
            break
        time.sleep(1)


# ---------------- LAND FUNCTION ----------------
def land_and_disarm(vehicle) -> None:
    """Initiates landing and waits until disarmed."""
    print("\n[STEP 6] Landing initiated...")
    vehicle.mode = VehicleMode("LAND")

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  → Altitude: {alt:.2f} m")
        if alt <= 0.2:
            print("✅ Landed successfully")
            break
        time.sleep(1)

    print("[STEP 7] Waiting for auto-disarm...")
    while vehicle.armed:
        print("  Waiting for disarm...")
        time.sleep(1)
    print("✅ Vehicle disarmed")


# ---------------- MAIN EXECUTION ----------------
if __name__ == "__main__":
    vehicle = None
    try:
        free_acm_usb_ports()
        fix_port_permissions()

        print("\n🔌 Connecting to flight controller...")
        vehicle, port, baud = detect_fc_connection()
        print(f"✅ Connected successfully on {port} @ {baud}")

        arm_and_takeoff(vehicle, TARGET_ALT)
        print("\n[HOVER] Holding position for 10 seconds...")
        time.sleep(10)
        land_and_disarm(vehicle)

    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        if vehicle is not None:
            vehicle.close()
        print("\n🔚 Done — disconnected from flight controller.")
