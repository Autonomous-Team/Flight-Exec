#!/usr/bin/env python3
"""Logger setup utility for drone operations."""

from __future__ import annotations

import logging
import os
import sys
from datetime import datetime


class RealTimeStreamHandler(logging.StreamHandler):
    """Stream handler that flushes immediately for real-time console output."""
    
    def emit(self, record):
        """Emit a record and flush immediately."""
        super().emit(record)
        self.flush()


def setup_logger(name: str = "drone_logger") -> logging.Logger:
    """Set up a logger with file and console handlers for real-time output.
    
    All logs are printed to console in real-time and saved to log files.
    
    Args:
        name: Logger name (default: "drone_logger")
        
    Returns:
        Configured logger instance
    """
    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)

    log_filename = datetime.now().strftime("drone_log_%Y-%m-%d_%H-%M-%S.log")
    log_path = os.path.join(log_dir, log_filename)

    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    # Remove existing handlers to avoid duplicates
    logger.handlers.clear()

    # File handler - saves all logs to file
    fh = logging.FileHandler(log_path)
    fh.setLevel(logging.DEBUG)

    # Console handler - prints all logs to console in real-time
    ch = RealTimeStreamHandler(sys.stdout)
    ch.setLevel(logging.DEBUG)

    # Formatter
    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(message)s",
        "%Y-%m-%d %H:%M:%S"
    )

    fh.setFormatter(formatter)
    ch.setFormatter(formatter)

    logger.addHandler(fh)
    logger.addHandler(ch)

    # Prevent propagation to root logger to avoid duplicate messages
    logger.propagate = False

    logger.info("Logger initialized.")
    logger.info(f"Log file: {log_path}")
    print(f"📝 All logs will be displayed in real-time and saved to: {log_path}", flush=True)

    return logger

