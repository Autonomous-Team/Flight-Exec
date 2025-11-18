#!/usr/bin/env python3
"""Shell command execution utilities."""

from __future__ import annotations

import logging
import subprocess
from typing import Tuple


def run_shell_cmd(cmd: str, timeout: float = 3.0) -> Tuple[int, str]:
    """
    Run a shell command (string) and return (returncode, stdout).
    
    Uses shell=True for globs (safe here because cmd is controlled).
    
    Args:
        cmd: Shell command to execute
        timeout: Maximum time to wait for command completion
    
    Returns:
        Tuple of (returncode, stdout)
    """
    try:
        logging.debug("Running shell command: %s", cmd)
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        logging.debug("Shell command rc=%s stdout=%s", res.returncode, res.stdout.strip())
        return res.returncode, res.stdout.strip()
    except subprocess.TimeoutExpired:
        logging.warning("Shell command timed out: %s", cmd)
        return 124, ""
    except Exception as e:
        logging.exception("Shell command error: %s", e)
        return 1, f"ERROR: {e}"

