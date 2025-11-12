#!/usr/bin/env python3
import collections
import logging
import time
import sys
import math
import subprocess
import serial
import serial.tools.list_ports
from dronekit import connect, VehicleMode, LocationGlobalRelative

# ---------------- FIX COLLECTIONS IMPORT FOR PYTHON 3.10+ ----------------
if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping

# ---------------- LOGGING ----------------
logging.getLogger("dronekit").setLevel(logging.CRITICAL)


# ---------------- FREE PORTS ----------------
def free_acm_usb_ports() -> None:
    """Attempt to find and kill processes using /dev/ttyACM* and /dev/ttyUSB* (unix-only)."""
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
        print("⚠️ 'lsof' not available on this system; skipping port-free step.")
    except Exception as e:
        print(f"⚠️ Could not check or free ports: {e}")


# ---------------- FIX PERMISSIONS ----------------
def fix_port_permissions():
    """Ensure /dev/ttyACM* and /dev/ttyUSB* have correct permissions."""
    print("\n🔐 Setting permissions for /dev/ttyACM* and /dev/ttyUSB* ports...")
    for base in ["ttyACM", "ttyUSB"]:
        for i in range(4):  # check first few devices
            port = f"/dev/{base}{i}"
            try:
                result = subprocess.run(["sudo", "chmod", "666", port],
                                        check=False, capture_output=True, text=True)
                if result.returncode == 0:
                    print(f"✅ Permissions set for {port} (rw for all users)")
                else:
                    print(f"⚠️  Could not set permissions for {port} (may not exist)")
            except Exception as e:
                print(f"  ❌ Error setting permissions for {port}: {e}")


# ---------------- DETECT FLIGHT CONTROLLER ----------------
def detect_fc_connection():
    """Automatically detect connected flight controller port and baud rate."""
    print("\n🔍 Searching for flight controller...")
    possible_ports = [p.device for p in serial.tools.list_ports.comports()
                      if "ACM" in p.device or "USB" in p.device]
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


# ---------------- GPS AUTO-DETECTION ----------------
def select_active_gps(vehicle):
    """Check available GPS units and return which one has the best fix."""
    print("\n[CHECK] Scanning GPS sources...")
    gps_sources = []
    for i in range(2):
        gps_attr = getattr(vehicle, f"gps_{i}", None)
        if gps_attr:
            gps_sources.append((i, gps_attr.fix_type, gps_attr.satellites_visible))
            print(f"  → GPS{i + 1}: fix_type={gps_attr.fix_type}, satellites={gps_attr.satellites_visible}")

    gps_sources = [g for g in gps_sources if g[1] >= 3]
    if not gps_sources:
        print("❌ No GPS fix yet. Waiting for any GPS to lock...")
        while True:
            for i in range(2):
                gps_attr = getattr(vehicle, f"gps_{i}", None)
                if gps_attr and gps_attr.fix_type >= 3:
                    print(
                        f"✅ Using GPS{i + 1} (fix_type={gps_attr.fix_type}, satellites={gps_attr.satellites_visible})")
                    return i
            time.sleep(1)
    else:
        best = max(gps_sources, key=lambda g: g[2])
        print(f"✅ Selected GPS{best[0] + 1} (fix_type={best[1]}, satellites={best[2]})")
        return best[0]


# ---------------- ARM AND TAKEOFF ----------------
def arm_and_takeoff(vehicle, target_altitude):
    print("\n[STEP 1] Checking pre-arm conditions...")
    while not vehicle.is_armable:
        print("  waiting for vehicle to initialize...")
        time.sleep(1)

    print("[STEP 2] Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("  waiting for mode change...")
        time.sleep(1)

    print("[STEP 3] Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("  waiting for arming...")
        time.sleep(1)
    print("✅ Vehicle armed")

    print(f"[STEP 4] Taking off to {target_altitude} meters at 0.1 m/s...")
    current_alt = vehicle.location.global_relative_frame.alt
    while current_alt < target_altitude:
        current_alt += 0.1
        vehicle.simple_takeoff(current_alt)
        print(f"  → Altitude: {current_alt:.2f} m")
        time.sleep(1)
        if current_alt >= target_altitude:
            break
    print("✅ Target altitude reached")


# ---------------- NAVIGATION HELPERS ----------------
def get_location_offset_meters(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    newlat = original_location.lat + (dNorth / earth_radius) * (180 / math.pi)
    newlon = original_location.lon + (dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))) * (
                180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * 1.113195e5) ** 2 + (dlong * 1.113195e5) ** 2)


def goto_point(vehicle, dNorth, dEast, target_altitude):
    print(
        f"\n[STEP 5] Going to target point ({dNorth} m North, {dEast} m East) at {target_altitude} m altitude (0.1 m/s)...")
    current_location = vehicle.location.global_relative_frame
    target_location = get_location_offset_meters(current_location, dNorth, dEast)
    target_location.alt = target_altitude
    vehicle.simple_goto(target_location, groundspeed=0.1)

    while True:
        dist = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"  → Distance: {dist:.2f} m | Altitude: {current_alt:.2f} m")
        if dist <= 0.5 and abs(current_alt - target_altitude) <= 0.2:
            print("✅ Arrived at target point and altitude")
            break
        time.sleep(1)


# ---------------- LAND AND DISARM ----------------
def gradual_descent_and_land(vehicle):
    print("\n[STEP 6] Initiating descent and landing at 0.1 m/s...")
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt <= 0.2:
            print("✅ Landed successfully")
            break
        alt -= 0.1
        print(f"  → Descending... current altitude: {alt:.2f} m")
        time.sleep(1)
    vehicle.mode = VehicleMode("LAND")
    print("[STEP 7] Disarming motors...")
    vehicle.armed = False
    while vehicle.armed:
        print("  waiting for disarm...")
        time.sleep(1)
    print("✅ Vehicle disarmed")


# ---------------- MAIN ----------------
if __name__ == "__main__":
    try:
        free_acm_usb_ports()
        fix_port_permissions()
        vehicle, port, baud = detect_fc_connection()
        print(f"📡 Using port {port} @ {baud} baud")

        active_gps_index = select_active_gps(vehicle)
        print(f"📍 Active GPS source: GPS{active_gps_index + 1}")

        if len(sys.argv) < 4:
            print("Usage: python3 drone_takeoff.py <takeoff_alt_m> <north_m> <east_m> <target_alt_m>")
            sys.exit(1)

        takeoff_alt = float(sys.argv[-4])
        north_dist = float(sys.argv[-3])
        east_dist = float(sys.argv[-2])
        target_alt = float(sys.argv[-1])

        arm_and_takeoff(vehicle, takeoff_alt)
        goto_point(vehicle, north_dist, east_dist, target_alt)
        time.sleep(3)
        gradual_descent_and_land(vehicle)
    finally:
        vehicle.close()
        print("\n🔚 Done — disconnected.")

