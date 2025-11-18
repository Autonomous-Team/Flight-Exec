"""Main entry points for flight operations."""

from main.arming_main import main as arming_main
from main.flight_main import main as flight_main
from main.mission_main import main as mission_main

__all__ = ["arming_main", "flight_main", "mission_main"]

