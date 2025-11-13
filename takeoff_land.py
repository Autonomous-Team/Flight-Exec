#!/usr/bin/env python3
import collections
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
TARGET_ALT = 5  # Target altitude in meters


# ---------------- FREE SERIAL PORTS ----------------
def free_acm_usb_ports() -> None:
    """Kill any process locking /dev/ttyACM* or /dev/ttyUSB*."""
    print("\n🧹 Checking for busy /dev/ttyACM* and /dev/ttyUSB* ports...")
    try:
        for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*"]:
            result = subprocess.run(["lsof", pattern], capture_output=True, text=True)
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
        print("⚠️ 'lsof' not available; skipping cleanup.")
    except Exception as e:
        print(f"⚠️ Could not check or free ports: {e}")


# ---------------- DETECT + PERMISSIONS ----------------
def detect_fc_and_prepare():
    """Detect connected flight controller with priority and 5 s timeout per port."""
    print("\n🔍 Detecting flight controller...")

    # Get all serial devices first
    detected_ports = [
        p.device for p in serial.tools.list_ports.comports()
        if "ACM" in p.device or "USB" in p.device
    ]
    if not detected_ports:
        raise RuntimeError("❌ No flight controller found on /dev/ttyACM* or /dev/ttyUSB*")

    # Prioritize ACM0 → ACM1 → USB0 → USB1 → others
    priority_order = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    ordered_ports = [p for p in priority_order if p in detected_ports] + [
        p for p in detected_ports if p not in priority_order
    ]

    print(f"Found ports (in priority order): {ordered_ports}")

    # Test with common baud rates, max 5 s timeout per attempt
    common_bauds = [115200, 57600, 921600]

    for port in ordered_ports:
        # Apply permission only to the current port
        try:
            subprocess.run(["sudo", "chmod", "666", port],
                           check=False, capture_output=True, text=True)
            print(f"🔐 Permissions set for {port}")
        except Exception as e:
            print(f"⚠️ Could not set permissions for {port}: {e}")

        for baud in common_bauds:
            try:
                print(f"  → Trying {port} @ {baud} (timeout 5s)...")
                vehicle = connect(port, baud=baud, wait_ready=True, timeout=5)
                print(f"✅ Connected successfully on {port} @ {baud}")
                return vehicle, port, baud
            except Exception:
                print(f"  ✖ {port} @ {baud} failed, trying next...")
                continue

    raise RuntimeError("❌ Could not connect to any detected port/baud combination.")


# ---------------- TAKEOFF FUNCTION ----------------
def arm_and_takeoff(vehicle, target_altitude: float) -> None:
    print("\n[STEP 1] Checking pre-arm conditions...")
    for _ in range(15):  # Wait up to 15 s for armable state
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


# ---------------- LAND FUNCTION ----------------
def land_and_disarm(vehicle) -> None:
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


# ---------------- MAIN ----------------
if __name__ == "__main__":
    vehicle = None
    try:
        free_acm_usb_ports()                     # Step 1: Free busy ports
        vehicle, port, baud = detect_fc_and_prepare()  # Step 2: Detect + connect (priority + timeout)

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
 