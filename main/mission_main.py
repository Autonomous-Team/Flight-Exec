#!/usr/bin/env python3
"""Main entry point for mission service."""

from __future__ import annotations

import argparse
from typing import Optional

from services.mission_service import execute_point_to_point_mission


def build_cli_parser() -> argparse.ArgumentParser:
    """Build CLI argument parser for mission service."""
    from config import get_mission_config

    cfg = get_mission_config()

    parser = argparse.ArgumentParser(
        description="Fly from takeoff to an offset waypoint, then land with comprehensive safety monitoring.",
    )
    parser.add_argument(
        "takeoff_altitude",
        type=float,
        nargs="?",
        help="Altitude to reach on takeoff (meters)",
    )
    parser.add_argument(
        "north_offset",
        type=float,
        nargs="?",
        help="Meters to travel north from the takeoff point",
    )
    parser.add_argument(
        "east_offset",
        type=float,
        nargs="?",
        help="Meters to travel east from the takeoff point",
    )
    parser.add_argument(
        "target_altitude",
        type=float,
        nargs="?",
        help="Cruise altitude at the waypoint (meters)",
    )
    parser.add_argument(
        "--hover-delay",
        type=float,
        default=None,
        help=f"Seconds to loiter before initiating landing (default: {cfg.hover_delay} from config)",
    )
    parser.add_argument(
        "--gps-wait",
        type=int,
        default=None,
        help=f"Seconds to wait for GPS fix if none available (default: {cfg.gps_wait_seconds} from config)",
    )
    parser.add_argument(
        "--no-safety-monitoring",
        action="store_true",
        help="Disable safety monitoring threads (NOT RECOMMENDED)",
    )
    return parser


def main(args: Optional[list[str]] = None) -> None:
    """CLI entry point for mission service."""
    from config import get_mission_config

    cfg = get_mission_config()

    parser = build_cli_parser()
    parsed = parser.parse_args(args=args)

    takeoff_altitude = (
        parsed.takeoff_altitude
        if parsed.takeoff_altitude is not None
        else cfg.default_takeoff_altitude
    )
    north_offset = (
        parsed.north_offset if parsed.north_offset is not None else cfg.default_north_offset
    )
    east_offset = (
        parsed.east_offset if parsed.east_offset is not None else cfg.default_east_offset
    )
    target_altitude = (
        parsed.target_altitude
        if parsed.target_altitude is not None
        else cfg.default_target_altitude
    )

    execute_point_to_point_mission(
        takeoff_altitude,
        north_offset,
        east_offset,
        target_altitude,
        hover_delay=parsed.hover_delay,
        gps_wait_seconds=parsed.gps_wait,
        enable_safety_monitoring=not parsed.no_safety_monitoring,
    )


if __name__ == "__main__":
    main()

