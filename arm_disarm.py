#!/usr/bin/env python3
import collections
if not hasattr(collections, "MutableMapping"):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
import time
import logging
import subprocess
import serial
import serial.tools.list_ports

# ---------------- LOGGING ----------------
logging.getLogger("dronekit").setLevel(logging.CRITICAL)


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

    detected_ports = [
        p.device for p in serial.tools.list_ports.comports()
        if "ACM" in p.device or "USB" in p.device
    ]
    if not detected_ports:
        raise RuntimeError("❌ No flight controller found on /dev/ttyACM* or /dev/ttyUSB*")

    priority_order = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    ordered_ports = [p for p in priority_order if p in detected_ports] + [
        p for p in detected_ports if p not in priority_order
    ]

    print(f"Found ports (in priority order): {ordered_ports}")
    common_bauds = [115200, 57600, 921600]

    for port in ordered_ports:
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


# ---------------- ARM / DISARM ----------------
def arm_vehicle(vehicle):
    """Force-arm motors without GPS (STABILIZE mode)."""
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


def disarm_vehicle(vehicle):
    """Disarm motors."""
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


# ---------------- MAIN ----------------
if __name__ == "__main__":
    vehicle = None
    try:
        free_acm_usb_ports()                      # Step 1: Free busy ports
        vehicle, port, baud = detect_fc_and_prepare()  # Step 2: Detect + connect (priority + timeout)

        arm_vehicle(vehicle)
        print("\n⏱ Holding armed state for 5 seconds...")
        time.sleep(5)
        disarm_vehicle(vehicle)

    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        if vehicle is not None:
            vehicle.close()
        print("\n🔚 Done — disconnected from flight controller.")
 