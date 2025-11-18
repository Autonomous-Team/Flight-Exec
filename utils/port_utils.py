#!/usr/bin/env python3
"""Serial port utilities for flight controller detection."""

from __future__ import annotations

import logging
import os
from typing import List

import serial.tools.list_ports

from utils.shell_utils import run_shell_cmd


def list_serial_ports() -> List[str]:
    """
    Return a list of candidate serial devices (ACM/USB) detected on system.
    
    Returns:
        List of serial port device paths
    """
    ports = [
        p.device
        for p in serial.tools.list_ports.comports()
        if ("ACM" in p.device or "USB" in p.device)
    ]
    logging.debug("Detected serial ports: %s", ports)
    return ports


def report_port_users() -> None:
    """
    Print processes using /dev/ttyACM* and /dev/ttyUSB* (do NOT kill them).
    
    Uses lsof if available. This is a safety feature to inform the user
    about potential port conflicts without automatically killing processes.
    """
    print("\n🧹 Checking for processes using serial ports (no killing will be performed)...")
    logging.info("Checking for processes using serial ports (no killing will be performed).")

    # Use shell globbing so lsof expands device patterns
    patterns = "/dev/ttyACM* /dev/ttyUSB*"
    rc, out = run_shell_cmd(f"lsof {patterns}", timeout=2.0)
    
    if rc == 0 and out:
        print("⚠️ The following processes are using serial ports:")
        print(out)
        logging.warning("lsof reported processes using serial ports:\n%s", out)
        print("  → Please stop interfering processes (e.g. mavproxy) before running this script.")
        logging.info("User instructed to stop interfering processes.")
    elif rc == 124:
        print("⚠️ 'lsof' timed out; skipping busy-port check.")
        logging.warning("'lsof' timed out; skipping busy-port check.")
    elif rc != 0 and "not found" in out.lower():
        print("⚠️ 'lsof' not found on system; install it or verify processes manually.")
        logging.warning("'lsof' not found on system; output: %s", out)
    else:
        print("✅ No processes reported by lsof for /dev/ttyACM* /dev/ttyUSB*.")
        logging.info("No processes reported by lsof for serial ports.")


def set_port_permissions(port: str) -> None:
    """
    Set permissions for a serial port if running as root.
    
    Args:
        port: Path to serial port device
    """
    if os.geteuid() == 0:
        try:
            os.chmod(port, 0o666)
            print(f"🔐 Permissions set for {port} (running as root)")
            logging.info("Permissions set for %s (running as root)", port)
        except Exception as e:
            print(f"⚠️ Could not chmod {port}: {e}. Continuing; user may have permissions already.")
            logging.warning("Could not chmod %s: %s", port, e)
    else:
        print(f"ℹ️ Not running as root. Ensure current user has permission to access {port} (udev rules or run with sudo).")
        logging.info("Not running as root; ensure permissions for %s", port)

