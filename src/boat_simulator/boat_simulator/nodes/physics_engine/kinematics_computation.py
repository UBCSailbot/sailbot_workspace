"""This module contains the kinematics computations for the boat."""

import numpy as np
from rclpy.logging import get_logger

import boat_simulator.common.constants as constants
from boat_simulator.common import utils
from boat_simulator.common.conventions import (
    NED,
    Acceleration,
    Body,
    Force,
    Inertia,
    InverseInertia,
    Position,
    Torque,
    Velocity,
)
from boat_simulator.common.types import Mat4, Vec4
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.kinematics_formulas import KinematicsFormulas

_logger = get_logger(__name__)


class BoatKinematics:
    """Computes and stores kinematic data of the boat at different time steps.

    Attributes:
        `timestep` (float): The time interval for calculations, expressed in seconds (s).
        `boat_mass` (float): The mass of the boat, expressed in kilograms (kg).
        `inertia` (Mat4[Inertia, Body]): The body-frame inertia tensor of the boat, expressed in
            kilograms-meters squared (kg•m^2).
        `inertia_inverse` (Mat4[Inertia, Body]): The inverse of the inertia tensor, expressed in
            per kilograms-meters squared (1/(kg•m^2)).
        `kinematics`: # TODO: Get this written down
    """

    def __init__(self, timestep: float, mass: float, inertia: Mat4[Inertia, Body]) -> None:
        """Initializes an instance of `BoatKinematics`.

        Args:
            timestep (float): The time interval for calculations, expressed in seconds (s).
            mass (float): The mass of the boat, expressed in kilograms (kg).
            inertia (Mat4[Inertia, Body]): The body-frame inertia tensor of the boat, expressed in
                kilograms-meters squared (kg•m^2).
        """
        self.__timestep = timestep
        self.__boat_mass = mass
        self.__inertia: Mat4[Inertia, Body] = inertia
        self.__inertia_inverse: Mat4[InverseInertia, Body] = Mat4(np.linalg.inv(inertia.data))
        self.kinematics: KinematicsData = KinematicsData()

    def step(self, glo_net_force: Vec4[Force, NED], net_torque: Vec4[Torque, Body]) -> None:
        """Updates the kinematic data based on applied forces and torques.

        Args:
            glo_net_force (Vec4[Force, NED]): The net force acting on the boat in the global
                reference frame, expressed in newtons (N).
            net_torque (Vec4[Torque, Body]): The net torque acting on the boat, expressed in
                newton-meters (N•m).

        Returns:
            None: The method updates the internal state of the boat's kinematics but does not
            return any data.
        """

        """
        3. BoatKinematics step:
            a. Run BoatKinematics.step()
                i. Calls KinematicsFormulas function is steps as shown below
                    1. KinematicsFormulas._compute_acceleration (F = m * a :) )
                    2. KinematicsFormulas._compute_velocity (v = v_initial + a * dt)
                    3. KinematicsFormulas._compute_transformation_and_position
                    4. KinematicsFormulas._compute_transformation: η̇ = J(η)·ν
                    5. KinematicsFormulas._compute_position: (n = n_initial + η̇  * dt)
                ii. Store the acceleration, velocity and position in the attributes
        Note: how is there going to be different timestamps when going from one timestamp to the
        next 😭
        """

        pass

    def __compute_acceleration(self):

        pass

    def __compute_velocity(self):

        pass

    def _compute_transformation_and_position(self):

        pass

    def _compute_position(self):

        pass

    @property
    def pose(self) -> Vec4[Position, NED]:
        return self.kinematics.pose

    @property
    def nu(self) -> Vec4[Velocity, Body]:
        return self.kinematics.nu

    @property
    def nu_dot(self) -> Vec4[Acceleration, Body]:
        return self.kinematics.nu_dot

    @property
    def timestep(self) -> float:
        return self.__timestep

    @property
    def inertia(self) -> Mat4[Inertia, Body]:
        return self.__inertia

    @property
    def inertia_inverse(self) -> Mat4[InverseInertia, Body]:
        return self.__inertia_inverse

    @property
    def boat_mass(self) -> float:
        return self.__boat_mass
