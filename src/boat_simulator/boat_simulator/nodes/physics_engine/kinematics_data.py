"""This module contains the kinematics data for the boat."""

from dataclasses import dataclass, field

from boat_simulator.common.conventions import (
    Acceleration,
    Position,
    Velocity,
)
from boat_simulator.common.types import Vec4
from boat_simulator.common.conventions import NED, Body


@dataclass
class KinematicsData():
    """Stores both linear and angular kinematic information pertaining to the boat.

    The generic ``Frame`` parameter specifies the reference frame of every stored
    vector.  Instantiate as ``KinematicsData[NED]`` for global-frame state or
    ``KinematicsData[Body]`` for body-frame state.


    Attributes:
        pose:    Pose [N, E, φ, ψ] in NED — metres (N, E) and radians (φ, ψ).
                 [N, E, φ, ψ] - north position, east position, roll angle, yaw angle (heading)
        nu:     Generalized body velocity [u, v, p, r] — m/s (u, v) and rad/s (p, r).
                [u, v, p, r] - surge vel, sway vel, roll rate, yaw rate
        nu_dot: Generalized body acceleration [u̇, v̇, ṗ, ṙ] — m/s² and rad/s².
                [u̇, v̇, ṗ, ṙ] - surge accel, sway accel, roll accel, yaw accel
    """

    pose: Vec4[Position, NED] = field(
        default_factory=lambda: Vec4.from_xypr(0.0, 0.0, 0.0, 0.0)
    )
    nu: Vec4[Velocity, Body] = field(
        default_factory=lambda: Vec4.from_xypr(0.0, 0.0, 0.0, 0.0)
    )
    nu_dot: Vec4[Acceleration, Body] = field(
        default_factory=lambda: Vec4.from_xypr(0.0, 0.0, 0.0, 0.0)
    )
