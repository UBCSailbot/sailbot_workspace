"""This module represents the state of the boat at a given step in time."""

from typing import Tuple

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.constants import BOAT_PROPERTIES
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData


class BoatState:
    """Represents the state of the boat at a specific point in time, including kinematic data
    in both relative and global reference frames.

    Attributes:
        `kinematics_computation` (BoatKinematics): The kinematic data for the boat in both
            the relative and global reference frames, used for computing future kinematic data,
            expressed in SI units.
    """

    def __init__(self, timestep: Scalar):
        """Initializes an instance of `BoatState`.

        Args:
            timestep (Scalar): The time interval for calculations, expressed in seconds (s).
        """
        self.__kinematics_computation = BoatKinematics(
            timestep, BOAT_PROPERTIES.mass, BOAT_PROPERTIES.inertia
        )

    def step(
        self,
        glo_wind_vel: NDArray,
        glo_water_vel: NDArray,
        rudder_angle_deg: Scalar,
        trim_tab_angle: Scalar,
    ) -> Tuple[KinematicsData, KinematicsData]:
        """Updates the boat's kinematic data based on applied forces and torques, and returns
        the updated kinematic data in both relative and global reference frames.

        Args:
            glo_wind_vel (NDArray): The velocity of the true wind in the global reference frame,
                expressed in meters per second (m/s).
            glo_water_vel (NDArray): The velocity of the current in the global reference frame,
                expressed in meters per second (m/s).
            rudder_angle_deg (float): The rudder angle with respect to the boat in degrees. Angle
                convention is 0 degrees is straight, increasing CCW.
            trim_tab_angle (float): The trim tab angle with respect to the wingsail in degrees.
                Angle convention is 0 degrees is straight, increasing CCW.

        Returns:
            Tuple[KinematicsData, KinematicsData]: A tuple with the first element representing
                kinematic data in the relative reference frame, and the second element representing
                data in the global reference frame, both using SI units.
        """
        rel_wind_vel = glo_wind_vel - self.global_velocity
        rel_water_vel = glo_water_vel - self.global_velocity

        rel_net_force, net_torque = self.__compute_net_force_and_torque(
            rel_wind_vel, rel_water_vel, rudder_angle_deg, trim_tab_angle
        )
        return self.__kinematics_computation.step(rel_net_force, net_torque)

    def __compute_net_force_and_torque(
        self,
        rel_wind_vel: NDArray,
        rel_water_vel: NDArray,
        rudder_angle_deg: Scalar,
        trim_tab_angle: Scalar,
    ) -> Tuple[NDArray, NDArray]:
        """Calculates the net force and net torque acting on the boat caused by the wind and water.

        Args:
            rel_wind_vel (NDArray): The velocity of the true wind in the relative reference frame,
                expressed in meters per second (m/s).
            rel_water_vel (NDArray): The velocity of the current in the relative reference frame,
                expressed in meters per second (m/s).
            rudder_angle_deg (float): The rudder angle with respect to the boat in degrees. Angle
                convention is 0 degrees is straight, increasing CCW.
            trim_tab_angle (float): The trim tab angle with respect to the wingsail in degrees.
                Angle convention is 0 degrees is straight, increasing CCW.

        Returns:
            Tuple[NDArray, NDArray]: A tuple where the first element represents the net force in
                the relative reference frame, expressed in newtons (N), and the second element
                represents the net torque, expressed in newton-meters (N•m).
        """
        # TODO Implement this function
        return (np.array([0, 0, 0]), np.array([0, 0, 0]))

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
