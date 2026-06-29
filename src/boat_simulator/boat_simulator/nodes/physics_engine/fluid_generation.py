"""This module provides a generator for fluid vectors used within the physics engine."""

import numpy as np
from rclpy.logging import get_logger

from boat_simulator.common.conventions import NED, Velocity
from boat_simulator.common.generators import VectorGenerator
from boat_simulator.common.types import Vec2
from boat_simulator.common.utils import bound_to_180, rad_to_degrees

_logger = get_logger(__name__)


class FluidGenerator:
    """This class provides functionality to generate velocity vectors representing fluid movements.

    Attributes:
        `generator` (VectorGenerator): The vector generator used to generate 2D fluid velocities.
        `velocity` (Vec2[Velocity, NED]): The most recently generated fluid velocity vector,
            expressed in meters per second (m/s).
    """

    def __init__(self, generator: VectorGenerator):
        self.__generator = generator
        self.__velocity: Vec2[Velocity, NED] = Vec2(self.__generator.next())

    def next(self) -> Vec2[Velocity, NED]:
        """Generates the next velocity vector for the fluid simulation.

        Returns:
            Vec2[Velocity, NED]: The updated fluid velocity vector in m/s.
        """
        self.__velocity = Vec2(self.__generator.next())
        _logger.debug(
            f"next: velocity={self.__velocity} speed={self.speed} direction={self.direction}"
        )
        return self.__velocity

    @property
    def velocity(self) -> Vec2[Velocity, NED]:
        """Returns the fluid's current velocity vector.

        Returns:
            Vec2[Velocity, NED]: The velocity vector of the fluid, expressed in
                meters per second (m/s) and ranging from negative infinity to positive infinity.
        """
        return self.__velocity

    @property
    def speed(self) -> float:
        """Calculates the current speed of the fluid.

        Returns:
            float: The speed of the fluid, expressed in meters per second (m/s) and within the
                range of 0 to positive infinity.
        """
        return float(np.linalg.norm(self.__velocity.data))

    @property
    def direction(self) -> float:
        """Calculates the current direction of the fluid.

        Returns:
            float: The direction of the fluid, expressed in degrees and bounded between
                [-180, 180).
        """
        angle_rad = np.arctan2(self.__velocity.data[1], self.__velocity.data[0])
        angle_deg = rad_to_degrees(angle_rad)
        return bound_to_180(angle_deg)
