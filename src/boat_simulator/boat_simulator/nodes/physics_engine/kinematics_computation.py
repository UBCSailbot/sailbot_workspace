"""This module contains the kinematics computations for the boat."""

import numpy as np
from rclpy.logging import get_logger

import boat_simulator.common.constants as constants
from boat_simulator.common import utils
from boat_simulator.common.conventions import (
    NED,
    Body,
    Force,
    Inertia,
    InverseInertia,
    Torque,
)
from boat_simulator.common.types import Mat3, Vec3
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.kinematics_formulas import KinematicsFormulas

_logger = get_logger(__name__)


class BoatKinematics:
    """Computes and stores kinematic data of the boat at different time steps.

    Attributes:
        `timestep` (float): The time interval for calculations, expressed in seconds (s).
        `boat_mass` (float): The mass of the boat, expressed in kilograms (kg).
        `inertia` (Mat3[Inertia, Body]): The body-frame inertia tensor of the boat, expressed in
            kilograms-meters squared (kg•m^2).
        `inertia_inverse` (Mat3[Inertia, Body]): The inverse of the inertia tensor, expressed in
            per kilograms-meters squared (1/(kg•m^2)).
        `relative_data` (KinematicsData): Kinematics data in the relative reference frame, using
            SI units.
        `global_data` (KinematicsData): Kinematics data in the global reference frame, using SI
            units.
    """

    def __init__(self, timestep: float, mass: float, inertia: Mat3[Inertia, Body]) -> None:
        """Initializes an instance of `BoatKinematics`.

        Args:
            timestep (float): The time interval for calculations, expressed in seconds (s).
            mass (float): The mass of the boat, expressed in kilograms (kg).
            inertia (Mat3[Inertia, Body]): The body-frame inertia tensor of the boat, expressed in
                kilograms-meters squared (kg•m^2).
        """
        self.__timestep = timestep
        self.__boat_mass = mass
        self.__inertia: Mat3[Inertia, Body] = inertia
        self.__inertia_inverse: Mat3[InverseInertia, Body] = Mat3(np.linalg.inv(inertia.data))
        self.__relative_data: KinematicsData[Body] = KinematicsData(is_relative=True)
        self.__global_data: KinematicsData[NED] = KinematicsData()

    def step(self, glo_net_force: Vec3[Force, NED], net_torque: Vec3[Torque, Body]) -> None:
        """Updates the kinematic data based on applied forces and torques.

        Args:
            glo_net_force (Vec3[Force, NED]): The net force acting on the boat in the global
                reference frame, expressed in newtons (N).
            net_torque (Vec3[Torque, Body]): The net torque acting on the boat, expressed in
                newton-meters (N•m).

        Returns:
            None: The method updates the internal state of the boat's kinematics but does not
            return any data.
        """

        yaw_radians = self.__update_ang_data(net_torque)
        # Express the NED force in the body frame before updating body-frame kinematics.
        orientation = self.global_data.angular_position
        ned_to_body = utils.ned_to_body_rotation_matrix(
            roll_rad=orientation.y,
            pitch_rad=orientation.x,
            yaw_rad=yaw_radians,
        )
        rel_net_force: Vec3[Force, Body] = Vec3(ned_to_body @ glo_net_force.data)
        self.__update_linear_relative_data(rel_net_force)

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

    def __update_ang_data(self, net_torque: Vec3[Torque, Body]) -> float:
        """Update the angular kinematics data.

        Args:
            net_torque (Vec3[Torque, Body]): The net torque acting on the boat, expressed in
                newton-meters (N•m).

        Returns:
            float: The next angular position along the yaw axis in the global reference frame,
                expressed in radians (rad).
        """

        # For this yaw-only planar model the angular acceleration, velocity, and position are
        # identical in the Body and NED frames (rotation is purely about the shared z axis), so the
        # results are stored into both KinematicsData[Body] and KinematicsData[NED]. They are typed
        # as frame-agnostic Vec3 to reflect that they are valid in either frame.
        next_ang_acceleration: Vec3 = Vec3(
            KinematicsFormulas.next_ang_acceleration(net_torque, self.inertia_inverse).data
        )

        next_ang_velocity: Vec3 = Vec3(
            KinematicsFormulas.next_velocity(
                self.global_data.angular_velocity,
                self.global_data.angular_acceleration,
                self.timestep,
            ).data
        )

        ang_pos = KinematicsFormulas.next_position(
            self.global_data.angular_position,
            self.global_data.angular_velocity,
            self.global_data.angular_acceleration,
            self.timestep,
        ).data
        # Wrap each angular component to [-π, π). Equivalent to bound_to_180(isDegrees=False), done
        # inline so the float64 angular vector is not narrowed by that helper's int32/float32 array
        # overload.
        ang_pos = (ang_pos + np.pi) % (2 * np.pi) - np.pi
        next_ang_position: Vec3 = Vec3(ang_pos)

        self.__relative_data.angular_acceleration = next_ang_acceleration
        self.__relative_data.angular_velocity = next_ang_velocity
        # The relative angular position is unused; KinematicsData forces it to zero.
        self.__relative_data.angular_position = next_ang_position

        self.__global_data.angular_acceleration = next_ang_acceleration
        self.__global_data.angular_velocity = next_ang_velocity
        self.__global_data.angular_position = next_ang_position

        yaw_radians = float(next_ang_position.data[constants.ORIENTATION_INDICES.YAW.value])

        _logger.debug(
            f"__update_ang_data: ang_acc={next_ang_acceleration} ang_vel={next_ang_velocity} "
            + f"ang_pos={next_ang_position} yaw={yaw_radians:.4f} rad"
        )

        return yaw_radians

    def __update_linear_relative_data(self, net_force: Vec3[Force, Body]) -> None:
        """Updates the linear kinematic data in the relative reference frame.

        Args:
            net_force (Vec3[Force, Body]): The net force acting on the boat in the relative (body)
                reference frame, expressed in newtons (N).
        """
        next_relative_acceleration: Vec3 = Vec3(
            KinematicsFormulas.next_lin_acceleration(self.boat_mass, net_force).data
        )
        next_relative_velocity = KinematicsFormulas.next_velocity(
            self.relative_data.linear_velocity,
            self.relative_data.linear_acceleration,
            self.timestep,
        )

        self.__relative_data.linear_acceleration = next_relative_acceleration
        self.__relative_data.linear_velocity = next_relative_velocity
        # The relative linear position is unused; KinematicsData forces it to zero.
        self.__relative_data.linear_position = Vec3.from_xyz(0.0, 0.0, 0.0)

        _logger.debug(
            f"__update_linear_relative_data: acc={next_relative_acceleration} "
            + f"vel={next_relative_velocity}"
        )

    def __update_linear_global_data(self, net_force: Vec3[Force, NED]) -> None:
        """Updates the linear kinematic data in the global reference frame.

        Args:
            net_force (Vec3[Force, NED]): The net force acting on the boat in the global reference
                frame, expressed in newtons (N).
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
            f"__update_linear_global_data: acc={next_global_acceleration} "
            + f"vel={next_global_velocity} pos={next_global_position}"
        )

    @property
    def relative_data(self) -> KinematicsData[Body]:
        return self.__relative_data

    @property
    def global_data(self) -> KinematicsData[NED]:
        return self.__global_data

    @property
    def timestep(self) -> float:
        return self.__timestep

    @property
    def inertia(self) -> Mat3[Inertia, Body]:
        return self.__inertia

    @property
    def inertia_inverse(self) -> Mat3[InverseInertia, Body]:
        return self.__inertia_inverse

    @property
    def boat_mass(self) -> float:
        return self.__boat_mass
