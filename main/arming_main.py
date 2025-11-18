#!/usr/bin/env python3
"""Main entry point for arming service."""

from __future__ import annotations

import argparse

from services.arming_service import execute_arm_disarm


def main() -> None:
    """CLI entry point for running the arm/disarm routine."""
    from config import get_arming_config

    cfg = get_arming_config()

    parser = argparse.ArgumentParser(
        description="Arm and disarm vehicle for ground testing (no GPS required)."
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=None,
        help=f"Seconds to hold armed state (default: {cfg.hold_seconds} from config, use 0 to skip)",
    )
    parser.add_argument(
        "--keep-checks",
        action="store_true",
        help="Keep arming checks enabled (default: disabled for ground testing)",
    )
    parser.add_argument(
        "--no-safety-monitoring",
        action="store_true",
        help="Disable safety monitoring threads (not recommended)",
    )

    args = parser.parse_args()

    # disable_checks logic: if --keep-checks is set, then disable_checks=False, else use config default
    disable_checks = None if not args.keep_checks else False

    execute_arm_disarm(
        hold_seconds=args.hold_seconds,
        disable_checks=disable_checks,
        enable_safety_monitoring=not args.no_safety_monitoring,
    )


if __name__ == "__main__":
    main()

