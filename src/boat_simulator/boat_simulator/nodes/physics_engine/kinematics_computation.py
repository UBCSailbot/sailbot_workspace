"""This module contains the kinematics computations for the boat."""

from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from rclpy.logging import get_logger

_logger = get_logger(__name__)

import boat_simulator.common.constants as constants
import boat_simulator.common.utils as utils
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.kinematics_formulas import KinematicsFormulas


class BoatKinematics:
    """Computes and stores kinematic data of the boat at different time steps.

    Attributes:
        `timestep` (Scalar): The time interval for calculations, expressed in seconds (s).
        `boat_mass` (Scalar): The mass of the boat, expressed in kilograms (kg).
        `inertia` (NDArray): The inertia of the boat, expressed in kilograms-meters squared
            (kg•m^2).
        `inertia_inverse` (NDArray): The inverse of the inertia matrix, expressed in
            kilograms-meters squared (kg•m^2).
        `relative_data` (KinematicsData): Kinematics data in the relative reference frame, using
            SI units.
        `global_data` (KinematicsData): Kinematics data in the global reference frame, using SI
            units.
    """

    def __init__(self, timestep: Scalar, mass: Scalar, inertia: NDArray) -> None:
        """Initializes an instance of `BoatKinematics`.

        Args:
            timestep (Scalar): The time interval for calculations, expressed in seconds (s).
            mass (Scalar): The mass of the boat, expressed in kilograms (kg).
            inertia (NDArray): The inertia of the boat, expressed in kilograms-meters squared
                (kg•m^2).
        """
        self.__timestep = timestep
        self.__boat_mass = mass
        assert inertia.shape == (3, 3)
        self.__inertia = inertia
        self.__inertia_inverse = np.linalg.inv(inertia)
        self.__relative_data = KinematicsData()
        self.__global_data = KinematicsData()

    def step(self, glo_net_force: NDArray, net_torque: NDArray) -> None:
        """Updates the kinematic data based on applied forces and torques.

        Args:
            rel_net_force (NDArray): The net force acting on the boat in the relative frame,
                expressed in newtons (N).
            net_torque (NDArray): The net torque acting on the boat, expressed in newton-meters
                (N•m).

        Returns:
            None: The method updates the internal state of the boat's kinematics but does not return any data.
        """

        yaw_radians = self.__update_ang_data(net_torque)

        self.__update_linear_relative_data(glo_net_force)

        # z-directional acceleration and velocity are neglected.
        # The net force from BoatState is already expressed in the global frame (it is computed
        # from global-frame apparent wind/water velocities and a global-convention orientation),
        # so it is used directly. The previous `rel_net_force * [cos(yaw), sin(yaw), 0]` was not a
        # valid rotation — it scaled and zeroed force components (e.g. forcing the y-force to 0
        # whenever yaw ≈ 0), which destroyed the velocity-squared drag that should oppose the
        # boat's motion and caused the apparent velocity (and forces) to diverge.
        self.__update_linear_global_data(glo_net_force)

        _logger.debug(
            f"step result: yaw={yaw_radians:.4f}rad rel_vel={self.relative_data.linear_velocity} "
            f"glo_pos={self.global_data.linear_position}"
        )

    def __update_ang_data(self, net_torque: NDArray) -> Scalar:
        """Update the angular kinematics data.

        Args:
            net_torque (NDArray): The net torque acting on the boat, expressed in newton-meters
                (N•m).

        Returns:
            Scalar: The next angular position along the yaw axis in the global reference frame,
                expressed in radians (rad).
        """

        next_ang_acceleration = KinematicsFormulas.next_ang_acceleration(
            net_torque, self.inertia_inverse
        )

        next_ang_velocity = KinematicsFormulas.next_velocity(
            self.global_data.angular_velocity,
            self.global_data.angular_acceleration,
            self.timestep,
        )

        next_ang_position = utils.bound_to_180(
            KinematicsFormulas.next_position(
                self.global_data.angular_position,
                self.global_data.angular_velocity,
                self.global_data.angular_acceleration,
                self.timestep,
            ),
            isDegrees=False,
        )

        self.__relative_data.angular_acceleration = next_ang_acceleration
        self.__relative_data.angular_velocity = next_ang_velocity
        self.__relative_data.angular_position[:] = 0  # relative angular position is unused

        self.__global_data.angular_acceleration = next_ang_acceleration
        self.__global_data.angular_velocity = next_ang_velocity
        self.__global_data.angular_position = next_ang_position

        yaw_radians = next_ang_position[constants.ORIENTATION_INDICES.YAW.value]

        _logger.debug(
            f"__update_ang_data: ang_acc={next_ang_acceleration} ang_vel={next_ang_velocity} ang_pos={next_ang_position} yaw={yaw_radians:.4f} rad"
        )

        return yaw_radians

    def __update_linear_relative_data(self, net_force: NDArray) -> None:
        """Updates the linear kinematic data in the relative reference frame.

        Args:
            net_force (NDArray): The net force acting on the boat in the relative reference
                frame, expressed in newtons (N).
        """
        next_relative_acceleration = KinematicsFormulas.next_lin_acceleration(
            self.boat_mass, net_force
        )
        next_relative_velocity = KinematicsFormulas.next_velocity(
            self.relative_data.linear_velocity,
            self.relative_data.linear_acceleration,
            self.timestep,
        )

        self.__relative_data.linear_acceleration = next_relative_acceleration
        self.__relative_data.linear_velocity = next_relative_velocity
        self.__relative_data.linear_position[:] = 0  # linear position is unused

        _logger.debug(
            f"__update_linear_relative_data: acc={next_relative_acceleration} vel={next_relative_velocity}"
        )

    def __update_linear_global_data(self, net_force: NDArray) -> None:
        """Updates the linear kinematic data in the global reference frame.

        Args:
            net_force (NDArray): The net force acting on the boat in the global reference frame,
                expressed in newtons (N).
        """
        next_global_acceleration = KinematicsFormulas.next_lin_acceleration(
            self.boat_mass, net_force
        )
        next_global_velocity = KinematicsFormulas.next_velocity(
            self.global_data.linear_velocity, self.global_data.linear_acceleration, self.timestep
        )
        next_global_position = KinematicsFormulas.next_position(
            self.global_data.linear_position,
            self.global_data.linear_velocity,
            self.global_data.linear_acceleration,
            self.timestep,
        )

        self.__global_data.linear_acceleration = next_global_acceleration
        self.__global_data.linear_velocity = next_global_velocity
        self.__global_data.linear_position = next_global_position

        _logger.debug(
            f"__update_linear_global_data: acc={next_global_acceleration} vel={next_global_velocity} pos={next_global_position}"
        )

    @property
    def relative_data(self) -> KinematicsData:
        return self.__relative_data

    @property
    def global_data(self) -> KinematicsData:
        return self.__global_data

    @property
    def timestep(self) -> Scalar:
        return self.__timestep

    @property
    def inertia(self) -> NDArray:
        return self.__inertia

    @property
    def inertia_inverse(self) -> NDArray:
        return self.__inertia_inverse

    @property
    def boat_mass(self) -> Scalar:
        return self.__boat_mass
