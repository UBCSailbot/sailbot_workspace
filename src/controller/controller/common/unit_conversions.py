"""Unit conversion helpers for the controller package."""


def kmph_to_mps(speed_kmph: float) -> float:
    """Convert a speed from kilometers per hour to meters per second."""
    return speed_kmph / 3.6
