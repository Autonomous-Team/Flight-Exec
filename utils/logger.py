#!/usr/bin/env python3
"""Logger setup utility for drone operations."""

from __future__ import annotations

import logging
import os
import sys
from datetime import datetime
from typing import Optional


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
        msg = record.getMessage()
        noisy_keywords = [
            "MAVLINK", "GPS", "GLOBAL_POSITION_INT", "HEARTBEAT", "ATTITUDE", "dronekit"
        ]
        # Hide records that contain any noisy keyword in message text
        return not any(key in str(msg) for key in noisy_keywords)


def _filehandler_already_added(root: logging.Logger, path: str) -> bool:
    """Check root handlers to avoid adding duplicate file handlers."""
    for h in root.handlers:
        try:
            if getattr(h, "baseFilename", None) == os.path.abspath(path):
                return True
        except Exception:
            continue
    return False


def setup_logger(name: str = "drone_logger") -> logging.Logger:
    """Configure logger with:
    - root file handler (DEBUG) to capture *all* loggers
    - a console handler attached only to `drone_logger` (INFO+, filtered)
    - dynamic file name: <scriptname>_<timestamp>.log
    """
    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)

    script_name = os.path.splitext(os.path.basename(sys.argv[0]))[0] or "script"
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_filename = f"{script_name}_{timestamp}.log"
    log_path = os.path.join(log_dir, log_filename)

    # Formatter for file includes logger name so you can see source (mavlink / dronekit / your module)
    file_formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] [%(name)s] %(message)s", "%Y-%m-%d %H:%M:%S"
    )
    console_formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s", "%Y-%m-%d %H:%M:%S"
    )

    # ---------- FILE HANDLER on ROOT (captures all loggers) ----------
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    if not _filehandler_already_added(root, log_path):
        fh = logging.FileHandler(log_path, encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(file_formatter)
        root.addHandler(fh)

    # ---------- CLEAN CONSOLE for your code only ----------
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)  # keep logger level low; handler controls what shows
    # Remove only handlers on this named logger (avoid removing root handlers)
    logger.handlers.clear()

    ch = RealTimeStreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    ch.addFilter(ConsoleFilter())
    ch.setFormatter(console_formatter)

    logger.addHandler(ch)
    # Do not propagate to root to avoid console duplicates from other libraries
    logger.propagate = False

    # Informational startup lines (writes to both file and console)
    logger.info("Logger initialized.")
    logger.info(f"Full log file: {os.path.abspath(log_path)}")
    # Small print for immediate user feedback on console
    print(f"📝 Clean logs shown here; full logs saved to: {os.path.abspath(log_path)}", flush=True)

    return logger
 