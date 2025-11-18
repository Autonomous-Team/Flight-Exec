#!/usr/bin/env python3
"""Compatibility helpers for legacy dependencies."""

from __future__ import annotations

import collections
import collections.abc


def ensure_dronekit_compat() -> None:
    """Ensure required collections aliases exist for dronekit on Python 3.10+."""
    if not hasattr(collections, "MutableMapping"):
        collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]


