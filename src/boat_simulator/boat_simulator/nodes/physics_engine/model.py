"""This module represents the state of the boat at a given step in time."""

from typing import Tuple

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.fluid_forces import MediumForceComputation


class BoatState:
    """Represents the state of the boat at a specific point in time, including kinematic data
    in both relative and global reference frames.

    Attributes:
        `kinematics_computation` (BoatKinematics): The kinematic data for the boat in both
            the relative and global reference frames, used for computing future kinematic data,
            expressed in SI units.
    """

    def __init__(self, timestep: Scalar, mass: Scalar, inertia: NDArray,
                 air_density: Scalar, water_density: Scalar):
        """Initializes an instance of `BoatState`.

        Args:
            timestep (Scalar): The time interval for calculations, expressed in seconds (s).
            mass (Scalar): The mass of the boat, expressed in kilograms (kg).
            inertia (NDArray): The inertia of the boat, expressed in kilograms-meters squared
                (kg•m^2).
            air_density(Scalar): Density of air, expressed in kilograms per meter cubed (kg/m^3)
            water_density(Scalar): Density of water, expressed in kilograms per meter cubed
                (kg/m^3)
        """
        self.__timestep = timestep
        self.__mass = mass
        self.__inertia = inertia
        self.__air_density = air_density
        self.__water_density = water_density

        self.__kinematics_computation = BoatKinematics(timestep, mass, inertia)
        self.__sail_force_computation = MediumForceComputation()

    def compute_net_force_and_torque(self, wind_vel: NDArray) -> Tuple[NDArray, NDArray]:
        """Calculates the net force and net torque acting on the boat due to the wind.

        Args:
            wind_vel (NDArray): The velocity of the wind, expressed in meters per second (m/s).

        Returns:
            Tuple[NDArray, NDArray]: A tuple where the first element represents the net force in
                the relative reference frame, expressed in newtons (N), and the second element
                represents the net torque, expressed in newton-meters (N•m).
        """
        raise NotImplementedError()

    def step(
        self, rel_net_force: NDArray, net_torque: NDArray
    ) -> Tuple[KinematicsData, KinematicsData]:
        """Updates the boat's kinematic data based on applied forces and torques, and returns
        the updated kinematic data in both relative and global reference frames.

        Args:
            rel_net_force (NDArray): The net force acting on the boat in the relative reference
            frame, expressed in newtons (N).
            net_torque (NDArray): The net torque acting on the boat, expressed in newton-meters
                (N•m).

        Returns:
            Tuple[KinematicsData, KinematicsData]: A tuple with the first element representing
                kinematic data in the relative reference frame, and the second element representing
                data in the global reference frame, both using SI units.
        """
        return self.__kinematics_computation.step(rel_net_force, net_torque)

    @property
    def global_position(self) -> NDArray:
        return self.__kinematics_computation.global_data.linear_position

    @property
    def global_velocity(self) -> NDArray:
        return self.__kinematics_computation.global_data.linear_velocity

    @property
    def global_acceleration(self) -> NDArray:
        return self.__kinematics_computation.global_data.linear_acceleration

    @property
    def relative_velocity(self) -> NDArray:
        return self.__kinematics_computation.relative_data.linear_velocity

    @property
    def relative_acceleration(self) -> NDArray:
        return self.__kinematics_computation.relative_data.linear_acceleration

    @property
    def angular_position(self) -> NDArray:
        return self.__kinematics_computation.relative_data.angular_position

    @property
    def angular_velocity(self) -> NDArray:
        return self.__kinematics_computation.relative_data.angular_velocity

    @property
    def angular_acceleration(self) -> NDArray:
        return self.__kinematics_computation.relative_data.angular_position

    @property
    def inertia(self) -> NDArray:
        return self.__kinematics_computation.inertia

    @property
    def inertia_inverse(self) -> NDArray:
        return self.__kinematics_computation.inertia_inverse

    @property
    def boat_mass(self) -> Scalar:
        return self.__kinematics_computation.boat_mass

    @property
    def timestep(self) -> Scalar:
        return self.__kinematics_computation.timestep

    @property
    def speed(self) -> Scalar:
        return np.linalg.norm(x=self.relative_velocity, ord=2)

    @property
    def true_bearing(self) -> Scalar:
        # TODO: Implement this function
        return 0
