"""Angle value objects and normalization helpers used by the simulator."""

from __future__ import annotations

import math
from dataclasses import dataclass

from rclpy.logging import get_logger

from boat_simulator.common.constants import RUDDER_MAX_ANGLE_RANGE, SAIL_MAX_ANGLE_RANGE

_LOGGER = get_logger("angle_conventions")

# Scalar helpers


def wrap_to_pi(angle: float) -> float:
    """Normalize an angle in radians to the half-open interval ``[-pi, pi)``."""
    if not math.isfinite(angle):
        raise _LOGGER.warn(f"angle must be finite, got {angle}")
    return (angle + math.pi) % math.tau - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp ``value`` to the inclusive interval ``[lower, upper]``."""
    if lower > upper:
        raise _LOGGER.warn(f"lower bound {lower} exceeds upper bound {upper}")
    return max(lower, min(upper, value))


# Angle value objects


@dataclass(frozen=True)
class Heading:
    """Heading in radians, normalized to ``[-pi, pi)``.

    This type deliberately does not prescribe clockwise/anticlockwise direction.
    Callers must convert at boundaries where the simulator's mathematical heading
    and ROS compass-heading conventions differ.
    """

    radians: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "radians", wrap_to_pi(self.radians))

    @classmethod
    def from_degrees(cls, degrees: float) -> Heading:
        return cls(math.radians(degrees))

    @property
    def degrees(self) -> float:
        return math.degrees(self.radians)


@dataclass
class RudderAngle:
    """Rudder steering angle in radians, limited to the simulator's ±30° range.

    This type is mutable: assigning to ``radians`` (e.g. after a computation)
    re-runs the range check, so the value can never be left outside the limits.
    """

    radians: float

    MIN_RAD = math.radians(RUDDER_MAX_ANGLE_RANGE[0])
    MAX_RAD = math.radians(RUDDER_MAX_ANGLE_RANGE[1])

    def __setattr__(self, name: str, value: float) -> None:
        if name == "radians":
            self._check_radians(value)
        super().__setattr__(name, value)

    def _check_radians(self, radians: float) -> None:
        if not math.isfinite(radians):
            raise _LOGGER.warn(f"steering angle must be finite, got {radians}")
        if not self.MIN_RAD <= radians <= self.MAX_RAD:
            raise _LOGGER.warn(
                "RudderAngle must be in "
                f"[{RUDDER_MAX_ANGLE_RANGE[0]} deg, {RUDDER_MAX_ANGLE_RANGE[1]} deg], "
                f"got {math.degrees(radians)} deg"
            )

    @classmethod
    def from_deg_to_rad(cls, degrees: float) -> RudderAngle:
        return cls(math.radians(degrees))

    @property
    def degrees(self) -> float:
        return math.degrees(self.radians)


@dataclass
class TrimTabAngle:
    """Trim-tab deflection angle in radians, limited to the configured sail range.

    The trim tab modifies the effective angle of attack of the sail foil.
    Pass ``.degrees`` to ``MediumForceComputation.compute()`` as the
    ``orientation`` argument, which expects angles in degrees.

    This type is mutable: assigning to ``radians`` (e.g. after a computation)
    re-runs the range check, so the value can never be left outside the limits.
    """

    radians: float

    MIN_RAD = math.radians(SAIL_MAX_ANGLE_RANGE[0])
    MAX_RAD = math.radians(SAIL_MAX_ANGLE_RANGE[1])

    def __setattr__(self, name: str, value: float) -> None:
        if name == "radians":
            self._check_radians(value)
        super().__setattr__(name, value)

    def _check_radians(self, radians: float) -> None:
        if not math.isfinite(radians):
            raise _LOGGER.warn(f"trim-tab angle must be finite, got {radians}")
        if not self.MIN_RAD <= radians <= self.MAX_RAD:
            raise _LOGGER.warn(
                "TrimTabAngle must be in "
                f"[{SAIL_MAX_ANGLE_RANGE[0]} deg, {SAIL_MAX_ANGLE_RANGE[1]} deg], "
                f"got {math.degrees(radians)} deg"
            )

    @classmethod
    def from_deg_to_rad(cls, degrees: float) -> TrimTabAngle:
        return cls(math.radians(degrees))

    @property
    def degrees(self) -> float:
        return math.degrees(self.radians)


def saturated_steering_angle(raw_radians: float) -> RudderAngle:
    """Construct a :class:`RudderAngle`, saturating finite input at the rudder limits."""
    if not math.isfinite(raw_radians):
        raise _LOGGER.warn(f"steering angle must be finite, got {raw_radians}")
    return RudderAngle(clamp(raw_radians, RudderAngle.MIN_RAD, RudderAngle.MAX_RAD))


def saturated_trim_tab_angle(raw_radians: float) -> TrimTabAngle:
    """Construct a :class:`TrimTabAngle`, saturating finite input at the trim-tab limits."""
    if not math.isfinite(raw_radians):
        raise _LOGGER.warn(f"trim-tab angle must be finite, got {raw_radians}")
    return TrimTabAngle(clamp(raw_radians, TrimTabAngle.MIN_RAD, TrimTabAngle.MAX_RAD))
