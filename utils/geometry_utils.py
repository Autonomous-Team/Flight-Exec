#!/usr/bin/env python3
"""Geometric calculation utilities for flight operations."""

from __future__ import annotations

import logging
import math


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Return distance in meters between two lat/lon points using the haversine formula.
    
    Args:
        lat1: Latitude of first point in degrees
        lon1: Longitude of first point in degrees
        lat2: Latitude of second point in degrees
        lon2: Longitude of second point in degrees
    
    Returns:
        Distance in meters
    """
    R = 6371000.0  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    dist = R * c
    
    logging.debug("Haversine: (%s,%s) to (%s,%s) = %s m", lat1, lon1, lat2, lon2, dist)
    return dist


def location_offset_meters(
    original_lat: float,
    original_lon: float,
    d_north: float,
    d_east: float,
) -> tuple[float, float]:
    """
    Return a new location offset by meters north/east.
    
    Args:
        original_lat: Original latitude in degrees
        original_lon: Original longitude in degrees
        d_north: Meters to offset north (positive) or south (negative)
        d_east: Meters to offset east (positive) or west (negative)
    
    Returns:
        Tuple of (new_lat, new_lon) in degrees
    """
    earth_radius = 6_378_137.0
    new_lat = original_lat + (d_north / earth_radius) * (180 / math.pi)
    new_lon = original_lon + (
        d_east / (earth_radius * math.cos(math.pi * original_lat / 180))
    ) * (180 / math.pi)
    return (new_lat, new_lon)

