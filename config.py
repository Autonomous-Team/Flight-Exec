#!/usr/bin/env python3
"""Configuration file for flight operations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class FlightConfig:
    """Flight operation configuration."""

    # Flight parameters
    target_altitude: float = 5.0  # meters
    hold_seconds: float = 10.0  # seconds to hold position
    goto_threshold_m: float = 0.5  # meters - only re-send goto if > this distance
    hold_rate_hz: float = 1.0  # Hz - send simple_goto at this rate when holding

    # Connection parameters
    fc_connect_timeout: float = 5.0  # seconds per connection attempt
    home_wait_timeout: int = 15  # seconds to wait for valid location after takeoff

    # Safety thresholds
    heartbeat_warn_sec: int = 5  # warn if heartbeat older than this
    heartbeat_critical_sec: int = 15  # consider connection lost if older than this
    battery_critical_threshold: float = 10.0  # trigger safe landing if battery below this %
    battery_low_threshold: float = 20.0  # warn if battery below this %

    # Monitoring
    enable_battery_monitor: bool = True
    enable_heartbeat_monitor: bool = True
    enable_jetson_monitor: bool = True
    enable_fc_telemetry_monitor: bool = True

    # Logging
    logging_detailed: bool = True  # Use DEBUG level logging
    logging_dir: str = "logs"  # Directory for log files


@dataclass
class ArmingConfig:
    """Arming service configuration."""

    # Arming parameters
    hold_seconds: float = 5.0  # seconds to hold armed state
    disable_checks: bool = True  # disable arming checks for ground testing

    # Connection parameters
    fc_connect_timeout: float = 5.0  # seconds per connection attempt

    # Safety thresholds
    heartbeat_warn_sec: int = 5
    heartbeat_critical_sec: int = 15
    battery_critical_threshold: float = 10.0
    battery_low_threshold: float = 20.0

    # Monitoring
    enable_battery_monitor: bool = True
    enable_heartbeat_monitor: bool = True
    enable_jetson_monitor: bool = True
    enable_fc_telemetry_monitor: bool = True

    # Logging
    logging_detailed: bool = True
    logging_dir: str = "logs"


@dataclass
class MissionConfig:
    """Mission service configuration."""

    # Mission parameters
    hover_delay: float = 3.0  # seconds to loiter at waypoint before landing
    gps_wait_seconds: int = 10  # seconds to wait for GPS fix
    goto_threshold_m: float = 0.5  # meters - distance threshold for waypoint arrival
    groundspeed: float = 0.1  # m/s - default groundspeed for navigation
    default_takeoff_altitude: float = 5.0  # meters
    default_north_offset: float = 0.0  # meters
    default_east_offset: float = 0.0  # meters
    default_target_altitude: float = 5.0  # meters

    # Connection parameters
    fc_connect_timeout: float = 5.0
    home_wait_timeout: int = 15

    # Safety thresholds
    heartbeat_warn_sec: int = 5
    heartbeat_critical_sec: int = 15
    battery_critical_threshold: float = 10.0
    battery_low_threshold: float = 20.0

    # Monitoring
    enable_battery_monitor: bool = True
    enable_heartbeat_monitor: bool = True
    enable_jetson_monitor: bool = True
    enable_fc_telemetry_monitor: bool = True

    # Logging
    logging_detailed: bool = True
    logging_dir: str = "logs"


# Global config instances (can be overridden)
_flight_config: Optional[FlightConfig] = None
_arming_config: Optional[ArmingConfig] = None
_mission_config: Optional[MissionConfig] = None


def get_flight_config() -> FlightConfig:
    """Get flight configuration (creates default if not set)."""
    global _flight_config
    if _flight_config is None:
        _flight_config = FlightConfig()
    return _flight_config


def get_arming_config() -> ArmingConfig:
    """Get arming configuration (creates default if not set)."""
    global _arming_config
    if _arming_config is None:
        _arming_config = ArmingConfig()
    return _arming_config


def get_mission_config() -> MissionConfig:
    """Get mission configuration (creates default if not set)."""
    global _mission_config
    if _mission_config is None:
        _mission_config = MissionConfig()
    return _mission_config


def set_flight_config(config: FlightConfig) -> None:
    """Set flight configuration."""
    global _flight_config
    _flight_config = config


def set_arming_config(config: ArmingConfig) -> None:
    """Set arming configuration."""
    global _arming_config
    _arming_config = config


def set_mission_config(config: MissionConfig) -> None:
    """Set mission configuration."""
    global _mission_config
    _mission_config = config

