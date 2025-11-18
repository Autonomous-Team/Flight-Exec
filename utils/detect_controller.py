#!/usr/bin/env python3
"""Detection and connection logic for ArduPilot-compatible controllers."""

from __future__ import annotations

import logging
from typing import Iterable, Sequence, Tuple

from utils.compat import ensure_dronekit_compat

ensure_dronekit_compat()

from dronekit import Vehicle, connect

from utils.port_utils import list_serial_ports, set_port_permissions

# Silence verbose dronekit logs by default; callers can override this logger.
logging.getLogger("dronekit").setLevel(logging.CRITICAL)

# Ordered preference for serial devices.
DEFAULT_PRIORITY_PORTS: Tuple[str, ...] = (
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
)

# Common baud rates to attempt when establishing a connection.
DEFAULT_BAUDS: Tuple[int, ...] = (115200, 57600, 921600)


def list_candidate_ports(
    priority_ports: Sequence[str] = DEFAULT_PRIORITY_PORTS,
) -> Tuple[str, ...]:
    """Return detected flight-controller ports ordered by priority."""
    print("\n🔍 Detecting flight controller...")
    detected_ports = list_serial_ports()

    if not detected_ports:
        raise RuntimeError("❌ No flight controller found on /dev/ttyACM* or /dev/ttyUSB*")

    ordered_ports = _order_ports(detected_ports, priority_ports)
    print(f"Found ports (in priority order): {ordered_ports}")
    return ordered_ports


def connect_to_first_available(
    ordered_ports: Sequence[str],
    *,
    preferred_bauds: Sequence[int] = DEFAULT_BAUDS,
    timeout_seconds: float = 5.0,
) -> Tuple[Vehicle, str, int]:
    """Attempt to connect to the first responsive port in the provided list."""
    for port in ordered_ports:
        set_port_permissions(port)

        for baud in preferred_bauds:
            try:
                print(f"  → Trying {port} @ {baud} (timeout {timeout_seconds}s)...")
                vehicle = connect(
                    port,
                    baud=baud,
                    wait_ready=True,
                    timeout=timeout_seconds,
                )
                print(f"✅ Connected successfully on {port} @ {baud}")
                return vehicle, port, baud
            except Exception:
                print(f"  ✖ {port} @ {baud} failed, trying next...")
                continue

    raise RuntimeError("❌ Could not connect to any detected port/baud combination.")


def detect_fc_and_prepare(
    *,
    priority_ports: Sequence[str] = DEFAULT_PRIORITY_PORTS,
    preferred_bauds: Sequence[int] = DEFAULT_BAUDS,
    timeout_seconds: float = 5.0,
) -> Tuple[Vehicle, str, int]:
    """Legacy helper that detects and connects to the first responsive controller."""
    ordered_ports = list_candidate_ports(priority_ports)
    return connect_to_first_available(
        ordered_ports,
        preferred_bauds=preferred_bauds,
        timeout_seconds=timeout_seconds,
    )


def _order_ports(
    detected_ports: Iterable[str],
    priority_ports: Sequence[str],
) -> Tuple[str, ...]:
    """Return detected ports sorted by configured priority."""
    prioritized = [port for port in priority_ports if port in detected_ports]
    remaining = [port for port in detected_ports if port not in priority_ports]
    return tuple(prioritized + remaining)


