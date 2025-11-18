#!/usr/bin/env python3
"""Flight controller telemetry monitoring service."""

from __future__ import annotations

import logging
import math
import threading

from dronekit import Vehicle

from services.safety_manager import is_safe_landing_triggered, trigger_safe_landing


def start_fc_telemetry_monitor(vehicle: Vehicle, stop_event: threading.Event) -> threading.Thread:
    """
    Start a background thread that periodically logs comprehensive FC telemetry.
    
    Args:
        vehicle: Connected vehicle instance
        stop_event: Event to signal thread to stop
    
    Returns:
        The Thread object
    """
    def monitor():
        while not stop_event.is_set():
            try:
                if not getattr(vehicle, "connected", False):
                    stop_event.wait(1.0)
                    continue

                # GPS status
                gps = getattr(vehicle, "gps_0", None)
                if gps is not None:
                    fix_type = getattr(gps, "fix_type", None)
                    satellites_visible = getattr(gps, "satellites_visible", None)
                    if fix_type is not None:
                        logging.info("FC: GPS fix_type=%d", fix_type)
                    if satellites_visible is not None:
                        logging.info("FC: GPS satellites_visible=%d", satellites_visible)

                # Attitude
                attitude = getattr(vehicle, "attitude", None)
                if attitude is not None:
                    roll = getattr(attitude, "roll", None)
                    pitch = getattr(attitude, "pitch", None)
                    yaw = getattr(attitude, "yaw", None)
                    if roll is not None and pitch is not None and yaw is not None:
                        logging.info(
                            "FC: Attitude roll=%.2f° pitch=%.2f° yaw=%.2f°",
                            math.degrees(roll),
                            math.degrees(pitch),
                            math.degrees(yaw),
                        )

                # Velocity
                velocity = getattr(vehicle, "velocity", None)
                if velocity is not None:
                    vx = getattr(velocity, "vx", None)
                    vy = getattr(velocity, "vy", None)
                    vz = getattr(velocity, "vz", None)
                    if vx is not None and vy is not None and vz is not None:
                        speed = math.sqrt(vx**2 + vy**2 + vz**2)
                        logging.info(
                            "FC: Velocity vx=%.2f vy=%.2f vz=%.2f speed=%.2f m/s", vx, vy, vz, speed
                        )

                # Heading
                heading = getattr(vehicle, "heading", None)
                if heading is not None:
                    logging.info("FC: Heading=%.1f°", heading)

                # Groundspeed and airspeed
                groundspeed = getattr(vehicle, "groundspeed", None)
                if groundspeed is not None:
                    logging.info("FC: Groundspeed=%.2f m/s", groundspeed)
                airspeed = getattr(vehicle, "airspeed", None)
                if airspeed is not None:
                    logging.info("FC: Airspeed=%.2f m/s", airspeed)

                # EKF status
                ekf_ok = getattr(vehicle, "ekf_ok", None)
                if ekf_ok is not None:
                    logging.info("FC: EKF OK=%s", ekf_ok)
                    if not ekf_ok:
                        logging.critical("FC: EKF FAILED - triggering safe landing")
                        trigger_safe_landing("EKF (Extended Kalman Filter) failed")

                # System status
                system_status = getattr(vehicle, "system_status", None)
                if system_status is not None:
                    state = getattr(system_status, "state", None)
                    if state is not None:
                        logging.info("FC: System status state=%s", state)

                # Mode
                mode = getattr(vehicle, "mode", None)
                if mode is not None:
                    mode_name = getattr(mode, "name", None)
                    if mode_name is not None:
                        logging.debug("FC: Mode=%s", mode_name)

            except Exception as e:
                logging.exception("FC telemetry monitor error: %s", e)

            # Check every 2 seconds (frequent enough for real-time telemetry)
            stop_event.wait(2.0)

    thr = threading.Thread(target=monitor, name="FCTelemetryMonitor", daemon=True)
    thr.start()
    logging.info("FC telemetry monitor thread started.")
    return thr

