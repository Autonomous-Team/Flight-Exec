#!/usr/bin/env python3
"""Logging configuration utilities for flight operations."""

from __future__ import annotations

import logging
import os
import sys
from datetime import datetime


def setup_logging(
    detailed: bool = True,
    log_dir: str = "logs",
    log_prefix: str = "flight",
) -> str:
    """
    Configure detailed logging to a timestamped file and also to console.
    
    Does not change existing print() behavior; logs are produced in parallel.
    
    Args:
        detailed: If True, use DEBUG level logging, else INFO
        log_dir: Directory for log files
        log_prefix: Prefix for log filename (e.g., "flight", "arming", "mission")
    
    Returns:
        Path to the log file that was created.
    """
    try:
        os.makedirs(log_dir, exist_ok=True)
    except Exception:
        # If directory creation fails, fallback to current directory
        log_dir = "."

    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_filename = os.path.join(log_dir, f"{log_prefix}_{ts}.log")

    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG if detailed else logging.INFO)

    # File handler (detailed)
    fh = logging.FileHandler(log_filename, encoding="utf-8")
    fh_level = logging.DEBUG if detailed else logging.INFO
    fh.setLevel(fh_level)
    fh_formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
    )
    fh.setFormatter(fh_formatter)
    logger.addHandler(fh)

    # Console handler (INFO+)
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    ch_formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", datefmt="%H:%M:%S")
    ch.setFormatter(ch_formatter)
    logger.addHandler(ch)

    # Also record the chosen file path via print (preserve prints)
    print(f"ℹ️ Logging initialized. Log file: {log_filename}")
    logging.info("Logging initialized. Log file: %s", log_filename)
    
    return log_filename

