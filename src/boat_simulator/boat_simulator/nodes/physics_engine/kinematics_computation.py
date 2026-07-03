"""This module contains the kinematics computations for the boat."""

import numpy as np
from rclpy.logging import get_logger

from boat_simulator.common.angle_conventions import wrap_to_pi
from boat_simulator.common.conventions import (
    NED,
    Acceleration,
    Body,
    BodytoNED,
    Force,
    Inertia,
    InverseInertia,
    Position,
    Torque,
    Transformer,
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
        `kinematics` (KinematicsData): The most recently computed pose η (NED), body velocity ν
            (Body), and body acceleration ν̇ (Body) of the boat.
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

    def step(self, net_force: Vec4[Force, Body], net_torque: Vec4[Torque, Body]) -> None:
        """Updates the kinematic data based on applied forces and torques.

        Args:
            net_force (Vec4[Force, Body]): The net force acting on the boat in the body
                reference frame, expressed in newtons (N).
            net_torque (Vec4[Torque, Body]): The net torque acting on the boat, expressed in
                newton-meters (N•m).

        Returns:
            None: The method updates the internal state of the boat's kinematics but does not
            return any data.
        """
        # Semi-implicit (symplectic) Euler: ν̇ and ν are updated first, then η̇ is computed from
        # the *updated* ν before η is integrated.
        self._compute_acceleration(net_force, net_torque)
        self._compute_velocity()
        self._compute_transformation_and_position()

        _logger.debug(f"step result: nu_dot={self.nu_dot} nu={self.nu} pose={self.pose}")

    def _compute_acceleration(
        self, net_force: Vec4[Force, Body], net_torque: Vec4[Torque, Body]
    ) -> None:
        """Computes ν̇ = [u̇, v̇, ṗ, ṙ] and stores it in `kinematics.nu_dot`.

        `net_force` carries the surge/sway force in the body frame;
        `net_torque` carries the roll/yaw moment, which is already expressed in the body frame,
        so K/I_x and N/I_z are obtained directly via
        `inertia_inverse`. Because the generalized inertia matrix is diagonal, summing the two
        contributions gives each DOF its own term without cross-coupling.
        """

        lin_acc = KinematicsFormulas.next_lin_acceleration(self.boat_mass, net_force)
        ang_acc = KinematicsFormulas.next_ang_acceleration(net_torque, self.inertia_inverse)

        self.kinematics.nu_dot = Vec4(lin_acc.data + ang_acc.data)

    def _compute_velocity(self) -> None:
        """Integrates ν ← ν + ν̇·dt and stores it in `kinematics.nu`."""
        self.kinematics.nu = KinematicsFormulas.next_velocity(self.nu, self.nu_dot, self.timestep)

    def _compute_transformation_and_position(self) -> None:
        """Computes η̇ = J(η)·ν, then integrates η ← η + η̇·dt."""
        eta_dot = self.__compute_transformation()
        self._compute_position(eta_dot)

    def __compute_transformation(self) -> Vec4[Velocity, NED]:
        """Maps the current body velocity ν to the world-frame pose rate η̇ = J(η)·ν:

            Ṅ = u·cos ψ − v·sin ψ
            Ė = u·sin ψ + v·cos ψ
            φ̇ = p
            ψ̇ = r·cos φ

        The cos(φ) term on the last row means this is not a plain yaw rotation (it degrades near
        φ = 90°, see the design doc's open items), so it is built by hand rather than reusing
        `Mat4.from_rotation_yaw`.
        """
        phi, psi = self.pose.z, self.pose.w
        cos_psi, sin_psi = np.cos(psi), np.sin(psi)
        jacobian: Mat4[Transformer, BodytoNED] = Mat4(
            np.array(
                [
                    [cos_psi, -sin_psi, 0.0, 0.0],
                    [sin_psi, cos_psi, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, np.cos(phi)],
                ]
            )
        )

        return Vec4(jacobian.data @ self.nu.data)

    def _compute_position(self, eta_dot: Vec4[Velocity, NED]) -> None:
        """Integrates η ← η + η̇·dt and wraps roll/yaw to [-π, π), storing the result in
        kinematics.pose.
        """

        next_pose = KinematicsFormulas.next_position(self.pose, eta_dot, self.timestep)

        wrapped = next_pose.data.copy()
        wrapped[2] = wrap_to_pi(wrapped[2])
        wrapped[3] = wrap_to_pi(wrapped[3])

        self.kinematics.pose = Vec4(wrapped)

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
