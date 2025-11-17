#!/usr/bin/env python3
"""Mission helpers for waypoint navigation flows."""

from __future__ import annotations

import argparse
import collections
import logging
import math
import time

from dronekit import LocationGlobalRelative, Vehicle, VehicleMode

from utils.detect_controller import connect_to_first_available, list_candidate_ports
from utils.free_ports import free_acm_usb_ports

if not hasattr(collections, "MutableMapping"):
    import collections.abc

    collections.MutableMapping = collections.abc.MutableMapping


def select_active_gps(vehicle: Vehicle, wait_seconds: int = 10) -> int:
    """Return the index of the GPS source with a valid fix.

    Args:
        vehicle: Connected dronekit vehicle instance.
        wait_seconds: Maximum time to wait for a GPS fix if none is available.

    Raises:
        RuntimeError: If no GPS fix is detected within the wait period.
    """
    print("\n[CHECK] Scanning GPS sources...")
    gps_sources = []
    for idx in range(2):
        gps_attr = getattr(vehicle, f"gps_{idx}", None)
        if gps_attr:
            gps_sources.append((idx, gps_attr.fix_type, gps_attr.satellites_visible))
            print(
                f"  → GPS{idx + 1}: fix_type={gps_attr.fix_type}, "
                f"satellites={gps_attr.satellites_visible}"
            )

    gps_sources = [entry for entry in gps_sources if entry[1] >= 3]
    if gps_sources:
        best_idx, fix_type, satellites = max(gps_sources, key=lambda item: item[2])
        print(f"✅ Selected GPS{best_idx + 1} (fix_type={fix_type}, satellites={satellites})")
        return best_idx

    print(f"❌ No GPS fix yet. Waiting for up to {wait_seconds} seconds for lock...")
    for _ in range(wait_seconds):
        for idx in range(2):
            gps_attr = getattr(vehicle, f"gps_{idx}", None)
            if gps_attr and gps_attr.fix_type >= 3:
                print(
                    f"✅ Using GPS{idx + 1} "
                    f"(fix_type={gps_attr.fix_type}, satellites={gps_attr.satellites_visible})"
                )
                return idx
        time.sleep(1)

    raise RuntimeError(f"❌ No GPS fix within {wait_seconds} seconds.")


def gradual_arm_and_takeoff(vehicle: Vehicle, target_altitude: float) -> None:
    """Arm the vehicle and take off gradually to the desired altitude."""
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


def goto_point(
    vehicle: Vehicle,
    d_north: float,
    d_east: float,
    target_altitude: float,
    groundspeed: float = 0.1,
) -> None:
    """Fly to a point offset from the current position."""
    print(
        f"\n[STEP 5] Going to target point ({d_north} m North, {d_east} m East) "
        f"at {target_altitude} m altitude ({groundspeed} m/s)..."
    )
    current_location = vehicle.location.global_relative_frame
    target_location = _location_offset_meters(current_location, d_north, d_east)
    target_location.alt = target_altitude
    vehicle.simple_goto(target_location, groundspeed=groundspeed)

    while True:
        distance = _distance_metres(vehicle.location.global_relative_frame, target_location)
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"  → Distance: {distance:.2f} m | Altitude: {current_alt:.2f} m")
        if distance <= 0.5 and abs(current_alt - target_altitude) <= 0.2:
            print("✅ Arrived at target point and altitude")
            break
        time.sleep(1)


def gradual_descent_and_land(vehicle: Vehicle) -> None:
    """Descend at 0.1 m/s until landing and disarm."""
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


def execute_point_to_point_mission(
    takeoff_altitude: float,
    north_offset: float,
    east_offset: float,
    target_altitude: float,
    *,
    hover_delay: float = 3.0,
    gps_wait_seconds: int = 10,
) -> None:
    """Run a complete point-to-point mission from takeoff to landing."""
    logging.getLogger("dronekit").setLevel(logging.CRITICAL)
    vehicle: Vehicle | None = None
    try:
        candidate_ports = list_candidate_ports()
        free_acm_usb_ports(candidate_ports)
        vehicle, port, baud = connect_to_first_available(candidate_ports)
        print(f"📡 Connected on {port} @ {baud}")

        active_gps_index = select_active_gps(vehicle, wait_seconds=gps_wait_seconds)
        print(f"📍 Active GPS: GPS{active_gps_index + 1}")

        gradual_arm_and_takeoff(vehicle, takeoff_altitude)
        goto_point(vehicle, north_offset, east_offset, target_altitude)
        if hover_delay > 0:
            print(f"\n[HOVER] Holding position for {hover_delay:.0f} seconds...")
            time.sleep(hover_delay)
        gradual_descent_and_land(vehicle)
    finally:
        if vehicle is not None:
            vehicle.close()
        print("\n🔚 Done — disconnected.")


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fly from takeoff to an offset waypoint, then land.",
    )
    parser.add_argument("takeoff_altitude", type=float, help="Altitude to reach on takeoff (meters)")
    parser.add_argument("north_offset", type=float, help="Meters to travel north from the takeoff point")
    parser.add_argument("east_offset", type=float, help="Meters to travel east from the takeoff point")
    parser.add_argument("target_altitude", type=float, help="Cruise altitude at the waypoint (meters)")
    parser.add_argument(
        "--hover-delay",
        type=float,
        default=3.0,
        help="Seconds to loiter before initiating landing (default: 3)",
    )
    parser.add_argument(
        "--gps-wait",
        type=int,
        default=10,
        help="Seconds to wait for GPS fix if none available (default: 10)",
    )
    return parser


def main(args: list[str] | None = None) -> None:
    parser = build_cli_parser()
    parsed = parser.parse_args(args=args)
    execute_point_to_point_mission(
        parsed.takeoff_altitude,
        parsed.north_offset,
        parsed.east_offset,
        parsed.target_altitude,
        hover_delay=parsed.hover_delay,
        gps_wait_seconds=parsed.gps_wait,
    )


if __name__ == "__main__":
    main()

def _location_offset_meters(
    original_location: LocationGlobalRelative,
    d_north: float,
    d_east: float,
) -> LocationGlobalRelative:
    """Return a new location offset by metres north/east."""
    earth_radius = 6_378_137.0
    new_lat = original_location.lat + (d_north / earth_radius) * (180 / math.pi)
    new_lon = original_location.lon + (
        d_east / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    ) * (180 / math.pi)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)


def _distance_metres(
    location_a: LocationGlobalRelative,
    location_b: LocationGlobalRelative,
) -> float:
    """Distance between two coordinates in metres."""
    d_lat = location_b.lat - location_a.lat
    d_lon = location_b.lon - location_a.lon
    return math.sqrt((d_lat * 1.113195e5) ** 2 + (d_lon * 1.113195e5) ** 2)


