#!/usr/bin/env python3
"""System statistics utilities for Jetson monitoring."""

from __future__ import annotations

import logging
import os
from typing import Dict, Optional

try:
    import psutil

    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False
    logging.warning("psutil not available; Jetson monitoring will be limited")

from utils.shell_utils import run_shell_cmd


def get_jetson_stats() -> Dict:
    """
    Get Jetson system statistics (CPU, memory, temperature, GPU if available).
    
    Returns:
        Dictionary with system stats including:
        - cpu_percent, cpu_count
        - mem_total_gb, mem_used_gb, mem_percent
        - disk_total_gb, disk_used_gb, disk_percent
        - temperatures (dict of zone names to temps in C)
        - gpu_util_percent, gpu_mem_used_mb, gpu_mem_total_mb, gpu_temp_c
    """
    stats: Dict = {}

    try:
        if PSUTIL_AVAILABLE:
            # CPU usage
            stats["cpu_percent"] = psutil.cpu_percent(interval=0.1)
            stats["cpu_count"] = psutil.cpu_count()

            # Memory
            mem = psutil.virtual_memory()
            stats["mem_total_gb"] = mem.total / (1024**3)
            stats["mem_used_gb"] = mem.used / (1024**3)
            stats["mem_percent"] = mem.percent

            # Disk
            disk = psutil.disk_usage("/")
            stats["disk_total_gb"] = disk.total / (1024**3)
            stats["disk_used_gb"] = disk.used / (1024**3)
            stats["disk_percent"] = disk.percent
    except Exception as e:
        logging.debug("Error getting psutil stats: %s", e)

    # Jetson-specific temperature (try multiple methods)
    try:
        # Try /sys/class/thermal for Jetson temperatures
        thermal_zones = ["thermal_zone0", "thermal_zone1", "thermal_zone2"]
        temps: Dict[str, float] = {}
        for zone in thermal_zones:
            try:
                temp_file = f"/sys/class/thermal/{zone}/temp"
                if os.path.exists(temp_file):
                    with open(temp_file, "r") as f:
                        temp_millidegrees = int(f.read().strip())
                        temp_c = temp_millidegrees / 1000.0
                        zone_name = zone
                        # Try to get type
                        type_file = f"/sys/class/thermal/{zone}/type"
                        if os.path.exists(type_file):
                            with open(type_file, "r") as tf:
                                zone_name = tf.read().strip()
                        temps[zone_name] = temp_c
            except Exception:
                pass
        if temps:
            stats["temperatures"] = temps
    except Exception:
        pass

    # GPU stats (Jetson-specific via tegrastats or nvidia-smi if available)
    try:
        # Try nvidia-smi for GPU stats
        rc, out = run_shell_cmd(
            "nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits",
            timeout=1.0,
        )
        if rc == 0 and out:
            parts = out.strip().split(", ")
            if len(parts) >= 4:
                stats["gpu_util_percent"] = float(parts[0])
                stats["gpu_mem_used_mb"] = float(parts[1])
                stats["gpu_mem_total_mb"] = float(parts[2])
                stats["gpu_temp_c"] = float(parts[3])
    except Exception:
        pass

    return stats

