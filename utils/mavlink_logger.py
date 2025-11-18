#!/usr/bin/env python3
"""MAVLink message logging utility."""

from __future__ import annotations

import logging


def create_mavlink_logger(logger: logging.Logger | None = None) -> callable:
    """Create a MAVLink message logger callback function.
    
    Args:
        logger: Logger instance to use. If None, uses default logger.
        
    Returns:
        Callback function for MAVLink message logging
    """
    if logger is None:
        logger = logging.getLogger("drone_logger")
    
    def mavlink_logger(vehicle, name: str, message) -> None:
        """Log MAVLink messages.
        
        Args:
            vehicle: Vehicle instance (unused but required by dronekit)
            name: Message name
            message: Message object
        """
        try:
            logger.debug(f"MAVLINK [{name}] {message}")
        except Exception as e:
            logger.error(f"MAVLink logging error: {e}")
    
    return mavlink_logger

