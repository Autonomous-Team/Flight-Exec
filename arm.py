#!/usr/bin/env python3
import collections

if not hasattr(collections, 'MutableMapping'):
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


# ---------------- PORT CLEANUP ----------------
def free_acm_usb_ports() -> None:
    """Kill any process locking /dev/ttyACM* or /dev/ttyUSB*."""
    print("\n🧹 Checking for busy /dev/ttyACM* and /dev/ttyUSB* ports...")
    try:
        for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*"]:
            result = subprocess.run(["lsof", pattern],
                                    capture_output=True, text=True)
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
        print("⚠️ 'lsof' not available on this system; skipping port cleanup.")
    except Exception as e:
        print(f"⚠️ Could not check or free ports: {e}")


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


# ---------------- ARM / DISARM ----------------
def arm_vehicle(vehicle):
    """Force-arm motors without GPS."""
    print("\n[STEP 1] Setting mode to STABILIZE (no GPS needed)...")
    vehicle.mode = VehicleMode("STABILIZE")
    while vehicle.mode.name != "STABILIZE":
        print("  Waiting for mode change...")
        time.sleep(1)

    print("[STEP 2] Forcing arm (bypassing GPS checks)...")
    vehicle.parameters['ARMING_CHECK'] = 0
    vehicle.armed = True
    while not vehicle.armed:
        print("  Waiting for arming...")
        time.sleep(1)
    print("✅ Motors armed successfully!")


def disarm_vehicle(vehicle):
    """Disarm motors."""
    print("\n[STEP 3] Disarming motors...")
    vehicle.armed = False
    while vehicle.armed:
        print("  Waiting for disarm...")
        time.sleep(1)
    print("✅ Motors disarmed successfully!")


# ---------------- MAIN ----------------
if __name__ == "__main__":
    try:
        free_acm_usb_ports()
        fix_port_permissions()

        print("🔌 Connecting to flight controller...")
        vehicle, port, baud = detect_fc_connection()
        print(f"✅ Connected successfully on {port} @ {baud}")

        arm_vehicle(vehicle)
        print("\n⏱ Holding armed state for 5 seconds...")
        time.sleep(5)
        disarm_vehicle(vehicle)

    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        try:
            vehicle.close()
            print("\n🔚 Done — disconnected from flight controller.")
        except Exception:
            pass
