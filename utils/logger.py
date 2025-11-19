#!/usr/bin/env python3
"""Logger setup utility for drone operations."""

from __future__ import annotations

import logging
import os
import sys
from datetime import datetime


class RealTimeStreamHandler(logging.StreamHandler):
    """Stream handler with immediate flush."""
    def emit(self, record):
        super().emit(record)
        self.flush()


class ConsoleFilter(logging.Filter):
    """
    Filter to HIDE noisy categories from console.
    File logs still receive everything.
    """
    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.msg

        # Hide these from console only:
        noisy_keywords = [
            "MAVLINK",     # all MAVLink messages
            "GPS",         # GPS spam
            "GLOBAL_POSITION_INT",
            "HEARTBEAT",
            "ATTITUDE",
            "dronekit",    # internal dronekit warnings
        ]

        return not any(key in str(msg) for key in noisy_keywords)


def setup_logger(name: str = "drone_logger") -> logging.Logger:
    """Configure logger with:
    - file log (full DEBUG)
    - console log (INFO+ only, filtered)
    - dynamic file name: <scriptname>_<timestamp>.log
    """
    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)

    # Determine script name being executed
    script_name = os.path.splitext(os.path.basename(sys.argv[0]))[0]

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_filename = f"{script_name}_{timestamp}.log"
    log_path = os.path.join(log_dir, log_filename)

    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()

    # -----------------------------
    # FILE HANDLER (FULL LOGS)
    # -----------------------------
    fh = logging.FileHandler(log_path)
    fh.setLevel(logging.DEBUG)

    # -----------------------------
    # CONSOLE HANDLER (CLEAN LOGS)
    # -----------------------------
    ch = RealTimeStreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)  # Only INFO+ to console
    ch.addFilter(ConsoleFilter())  # Remove noisy internal logs

    # -----------------------------
    # FORMATTER
    # -----------------------------
    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s",
        "%Y-%m-%d %H:%M:%S",
    )

    fh.setFormatter(formatter)
    ch.setFormatter(formatter)

    logger.addHandler(fh)
    logger.addHandler(ch)

    logger.propagate = False

    logger.info("Logger initialized.")
    logger.info(f"Log file: {log_path}")

    print(f"📝 Clean logs shown here; full logs saved to: {log_path}", flush=True)

    return logger
