#!/usr/bin/env python3
"""Jetson system monitoring service for flight operations."""

from __future__ import annotations

import logging
import threading

from utils.system_stats import get_jetson_stats


def start_jetson_monitor(stop_event: threading.Event) -> threading.Thread:
    """
    Start a background thread that periodically logs Jetson system statistics.
    
    Args:
        stop_event: Event to signal thread to stop
    
    Returns:
        The Thread object
    """
    def monitor():
        while not stop_event.is_set():
            try:
                stats = get_jetson_stats()

                # Log CPU and memory
                if "cpu_percent" in stats:
                    logging.info(
                        "JETSON: CPU usage=%.1f%% (%d cores)",
                        stats["cpu_percent"],
                        stats.get("cpu_count", 0),
                    )
                if "mem_percent" in stats:
                    logging.info(
                        "JETSON: Memory usage=%.1f%% (%.2f/%.2f GB)",
                        stats["mem_percent"],
                        stats.get("mem_used_gb", 0),
                        stats.get("mem_total_gb", 0),
                    )
                if "disk_percent" in stats:
                    logging.info(
                        "JETSON: Disk usage=%.1f%% (%.2f/%.2f GB)",
                        stats["disk_percent"],
                        stats.get("disk_used_gb", 0),
                        stats.get("disk_total_gb", 0),
                    )

                # Log temperatures
                if "temperatures" in stats:
                    for zone, temp in stats["temperatures"].items():
                        logging.info("JETSON: Temperature %s=%.1f°C", zone, temp)
                        # Warn on high temperature
                        if temp > 80:
                            logging.warning("JETSON: High temperature warning - %s=%.1f°C", zone, temp)

                # Log GPU stats
                if "gpu_util_percent" in stats:
                    logging.info("JETSON: GPU usage=%.1f%%", stats["gpu_util_percent"])
                if "gpu_mem_used_mb" in stats:
                    logging.info(
                        "JETSON: GPU memory=%.0f/%.0f MB",
                        stats["gpu_mem_used_mb"],
                        stats.get("gpu_mem_total_mb", 0),
                    )
                if "gpu_temp_c" in stats:
                    logging.info("JETSON: GPU temperature=%.1f°C", stats["gpu_temp_c"])
                    if stats["gpu_temp_c"] > 80:
                        logging.warning(
                            "JETSON: High GPU temperature warning - %.1f°C", stats["gpu_temp_c"]
                        )

            except Exception as e:
                logging.exception("Jetson monitor error: %s", e)

            # Check every 5 seconds (system stats don't need to be as frequent)
            stop_event.wait(5.0)

    thr = threading.Thread(target=monitor, name="JetsonMonitor", daemon=True)
    thr.start()
    logging.info("Jetson system monitor thread started.")
    return thr

