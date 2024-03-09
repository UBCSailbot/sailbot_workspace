"""This module contains the kinematics data for the boat."""

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


@dataclass
class KinematicsData:
    """Stores both linear and angular kinematic information pertaining to the boat.

    Attributes:
        `linear_position` (NDArray): Linear position of the boat, expressed in meters (m).
        `linear_velocity` (NDArray): Linear velocity of the boat, expressed in meters per
            second (m/s).
        `linear_acceleration` (NDArray): Linear acceleration of the boat, expressed in
            meters per second squared (m/s^2).
        `angular_position` (NDArray): Angular position of the boat, expressed in radians
            (rad).
        `angular_velocity` (NDArray): Angular velocity of the boat, expressed in radians
            per second (rad/s).
        `angular_acceleration` (NDArray): Angular acceleration of the boat, expressed in
            radians per second squared (rad/s^2).
    """

    # TODO: Ensure position is always set to 0 for relative reference frame
    linear_position: NDArray = field(default=np.zeros(3, dtype=np.float32))
    linear_velocity: NDArray = field(default=np.zeros(3, dtype=np.float32))
    linear_acceleration: NDArray = field(default=np.zeros(3, dtype=np.float32))
    angular_position: NDArray = field(default=np.zeros(3, dtype=np.float32))
    angular_velocity: NDArray = field(default=np.zeros(3, dtype=np.float32))
    angular_acceleration: NDArray = field(default=np.zeros(3, dtype=np.float32))
