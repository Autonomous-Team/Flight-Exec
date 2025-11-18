#!/usr/bin/env python3
"""
Main flight execution script with comprehensive safety monitoring.

This script orchestrates all flight services including:
- Connection management
- Safety monitoring (battery, heartbeat, EKF, etc.)
- System monitoring (Jetson stats)
- Flight operations (takeoff, hold, land)
- Safe landing procedures

This file is kept for backward compatibility. The main implementation
is in main/flight_main.py.
"""

from __future__ import annotations

if __name__ == "__main__":
    # Import and run from main directory
    from main.flight_main import main

    main()

