"""This module provides a generator for fluid vectors used within the physics engine."""

from boat_simulator.common.utils import bound_to_180, rad_to_degrees
import numpy as np
import math
from numpy.typing import NDArray
from boat_simulator.common.generators import VectorGenerator
from boat_simulator.common.types import Scalar, ScalarOrArray


class FluidGenerator:
    """This class provides functionality to generate velocity vectors representing fluid movements.

    Attributes:
        `generator` (VectorGenerator): The vector generator used to generate fluid velocities.
        `velocity` (NDArray): The most recently generated fluid velocity vector, expressed in
            meters per second (m/s).
    """

    def __init__(self, generator: VectorGenerator):
        self.__generator = generator  # Vector generator instance to produce velocity vectors
        self.__velocity = np.array(self.__generator.next())  # Initial velocity vector of the fluid

    def next(self) -> NDArray:
        """Generates the next velocity vector for the fluid simulation.

        Returns: 
            NDArray: An array representing the updated velocity vector for the fluid simulation.
        """

        # TODO: Implement this to generate the next velocity vector.

        # Generate the next velocity vector using the provided vector generator
        self.__velocity = np.array(self.__generator.next())
        return self.__velocity

        raise NotImplementedError()

    @property
    def velocity(self) -> NDArray:
        """Returns the fluid's current velocity vector.

        Returns:
            NDArray: The velocity vector of the fluid, expressed in meters per second (m/s) and
                ranging from negative infinity to positive infinity.
        """
        
        # Return the current velocity vector
        return self.__velocity


        raise NotImplementedError()

    @property
    def speed(self) -> Scalar:
        """Calculates the current speed of the fluid.

        Returns:
            Scalar: The speed of the fluid, expressed in meters per second (m/s) and within the
                range of 0 to positive infinity.
        """

        # TODO: Implement this using the current velocity vector.

        # Calculate the magnitude (speed) of the velocity vector
        return np.linalg.norm(self.__velocity)
    
        raise NotImplementedError()

    @property
    def direction(self) -> Scalar:
        """Calculates the current direction of the fluid.

        Returns:
            Scalar: The direction of the fluid, expressed in degrees and bounded between
                [-180, 180).
        """

        # TODO: Implement this using the current velocity vector.

        # Ensure the velocity vector is suitable for direction calculation
        if self.__velocity.ndim > 1 or self.__velocity.size < 2:
            raise ValueError("Velocity vector must be a 2D vector for direction calculation.")
        # Calculate the angle of the velocity vector in radians, then convert to degrees and adjust to [-180, 180) range
        angle_rad = np.arctan2(self.__velocity[1], self.__velocity[0])
        angle_deg = rad_to_degrees(angle_rad)
        return bound_to_180(angle_deg)