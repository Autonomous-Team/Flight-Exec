#!/usr/bin/env python3
"""Utilities for freeing busy serial ports used by ArduPilot controllers."""

from __future__ import annotations

import subprocess
from typing import Sequence


def free_acm_usb_ports(target_ports: Sequence[str] | None = None) -> None:
    """Kill any process locking the provided serial ports or default ACM/USB devices."""
    port_patterns = list(target_ports) if target_ports else ["/dev/ttyACM*", "/dev/ttyUSB*"]

    print("\n🧹 Checking for busy ports...")
    try:
        for pattern in port_patterns:
            result = subprocess.run(
                ["lsof", pattern],
                capture_output=True,
                text=True,
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
        print("⚠️ 'lsof' not available; skipping cleanup.")
    except Exception as exc:  # pragma: no cover - defensive logging
        print(f"⚠️ Could not check or free ports: {exc}")


