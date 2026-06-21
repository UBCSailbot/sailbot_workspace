"""This module contains the kinematics data for the boat."""

from dataclasses import dataclass, field
from typing import Generic

from boat_simulator.common.frames import (
    Acceleration,
    Frame,
    Position,
    Vec3,
    Velocity,
)

_ZERO_POSITION: Vec3 = Vec3.from_xyz(0.0, 0.0, 0.0)


@dataclass
class KinematicsData(Generic[Frame]):
    """Stores both linear and angular kinematic information pertaining to the boat.

    The generic ``Frame`` parameter specifies the reference frame of every stored
    vector.  Instantiate as ``KinematicsData[NED]`` for global-frame state or
    ``KinematicsData[Body]`` for body-frame state.

    ``Position`` covers both metres (linear) and radians (angular).
    ``Velocity`` covers both m/s (linear) and rad/s (angular).
    ``Acceleration`` covers both m/s² (linear) and rad/s² (angular).

    Attributes:
        linear_position (Vec3[Position, Frame]): Linear position, metres.
        linear_velocity (Vec3[Velocity, Frame]): Linear velocity, m/s.
        linear_acceleration (Vec3[Acceleration, Frame]): Linear acceleration, m/s².
        angular_position (Vec3[Position, Frame]): Euler angles [pitch, roll, yaw], radians.
        angular_velocity (Vec3[Velocity, Frame]): Angular rates [p, q, r], rad/s.
        angular_acceleration (Vec3[Acceleration, Frame]): Angular accelerations, rad/s².
    """

    is_relative: bool = False
    linear_position: Vec3[Position, Frame] = field(
        default_factory=lambda: Vec3.from_xyz(0.0, 0.0, 0.0)
    )
    linear_velocity: Vec3[Velocity, Frame] = field(
        default_factory=lambda: Vec3.from_xyz(0.0, 0.0, 0.0)
    )
    linear_acceleration: Vec3[Acceleration, Frame] = field(
        default_factory=lambda: Vec3.from_xyz(0.0, 0.0, 0.0)
    )
    angular_position: Vec3[Position, Frame] = field(
        default_factory=lambda: Vec3.from_xyz(0.0, 0.0, 0.0)
    )
    angular_velocity: Vec3[Velocity, Frame] = field(
        default_factory=lambda: Vec3.from_xyz(0.0, 0.0, 0.0)
    )
    angular_acceleration: Vec3[Acceleration, Frame] = field(
        default_factory=lambda: Vec3.from_xyz(0.0, 0.0, 0.0)
    )

    def __setattr__(self, name: str, value: object) -> None:
        if name in ("linear_position", "angular_position") and getattr(
            self, "is_relative", False
        ):
            value = _ZERO_POSITION
        object.__setattr__(self, name, value)
