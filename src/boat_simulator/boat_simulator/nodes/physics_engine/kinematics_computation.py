"""This module contains the kinematics computations for the boat."""

import math

import numpy as np
from numpy.typing import NDArray
from rclpy.logging import get_logger

from boat_simulator.common.angle_conventions import wrap_to_pi
from boat_simulator.common.constants import BOAT_PROPERTIES
from boat_simulator.common.conventions import (
    NED,
    Acceleration,
    Body,
    BodytoNED,
    Force,
    Inertia,
    InverseInertia,
    Position,
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
        `inertia` (Mat4[Inertia, Body]): M_RB, the rigid-body generalized mass-inertia
            matrix, expressed in kilograms / kilograms-meters squared (kg / kg•m^2).
        `inertia_inverse` (Mat4[Inertia, Body]): (M_RB + M_A)⁻¹, the inverse of the total
            (rigid-body plus added-mass) generalized mass-inertia matrix that the equations
            of motion are solved with.
        `kinematics` (KinematicsData): The most recently computed pose η (NED), body velocity ν
            (Body), and body acceleration ν̇ (Body) of the boat.
    """

    def __init__(self, timestep: float) -> None:
        """Initializes an instance of `BoatKinematics`.

        Args:
            timestep (float): The time interval for calculations, expressed in seconds (s).
        """
        self.__timestep = timestep
        self.__boat_mass = BOAT_PROPERTIES.mass
        # BOAT_PROPERTIES.inertia is M_RB = diag(m, m, I_xx, I_zz), the rigid-body
        # generalized mass-inertia matrix. The added mass M_A is folded into the mass
        # matrix here (the left-hand side of the equations of motion) rather than applied
        # as a −M_A·ν̇ force: an explicit added-mass force needs the previous timestep's
        # acceleration, which is numerically unstable when M_A is comparable to M_RB.
        self.__inertia: Mat4[Inertia, Body] = BOAT_PROPERTIES.inertia
        self.__inertia_inverse: Mat4[InverseInertia, Body] = Mat4(
            np.linalg.inv(BOAT_PROPERTIES.inertia.data + BOAT_PROPERTIES.M_A.data)
        )
        self.kinematics: KinematicsData = KinematicsData()

    def step(self, nu: Vec4[Velocity, Body], net_force: Vec4[Force, Body]) -> None:
        """Updates the kinematic data based on the applied generalized force.

        Args:
            nu (Vec4[Velocity, Body]): ν, the boat's generalized velocity [u, v, p, r].
            net_force (Vec4[Force, Body]): τ_RB, the total generalized force [X, Y, K, N]
                acting on the boat in the body reference frame.

        Returns:
            None: The method updates the internal state of the boat's kinematics but does not
            return any data.
        """
        # Semi-implicit (symplectic) Euler: ν̇ and ν are updated first, then η̇ is computed from
        # the *updated* ν before η is integrated.
        self.kinematics.nu_dot = self._compute_acceleration(nu, net_force)
        self._compute_velocity()
        self._compute_transformation_and_position()

        _logger.debug(f"step result: nu_dot={self.nu_dot} nu={self.nu} pose={self.pose}")

    def _compute_acceleration(
        self, nu: Vec4[Velocity, Body], tau_rb: Vec4[Force, Body]
    ) -> Vec4[Acceleration, Body]:
        """Solves the kinetics equation for the body-frame acceleration:
        ν̇ = (M_RB + M_A)⁻¹·(τ_RB − C_RB(ν)·ν).

        Args:
            nu (Vec4[Velocity, Body]): ν, the boat's generalized velocity [u, v, p, r].
            tau_rb (Vec4[Force, Body]): τ_RB, the total generalized force.

        Returns:
            Vec4[Acceleration, Body]: ν̇ = [u̇, v̇, ṗ, ṙ], the generalized acceleration.
        """
        coriolis_term = self.rigid_body_coriolis_matrix(nu) @ nu.data
        return Vec4(self.__inertia_inverse.data @ (tau_rb.data - coriolis_term))

    def rigid_body_coriolis_matrix(self, nu: Vec4[Velocity, Body]) -> NDArray[np.float64]:
        """Builds C_RB(ν), the rigid-body Coriolis-centripetal matrix.

        With the body origin at the centre of gravity, only the surge-sway-yaw coupling
        terms survive in the 4-DOF model::

            C_RB(ν) = ⎡  0    0   0  −m·v ⎤
                      ⎢  0    0   0   m·u ⎥
                      ⎢  0    0   0    0  ⎥
                      ⎣ m·v −m·u  0    0  ⎦

        Args:
            nu (Vec4[Velocity, Body]): ν, the boat's generalized velocity [u, v, p, r].

        Returns:
            NDArray[np.float64]: the 4x4 C_RB(ν) matrix.
        """
        m_x = self.__inertia.data[0, 0]
        m_y = self.__inertia.data[1, 1]
        u, v = nu.x, nu.y
        c_rb = np.zeros((4, 4))
        c_rb[0, 3] = -m_y * v
        c_rb[1, 3] = m_x * u
        c_rb[3, 0] = m_y * v
        c_rb[3, 1] = -m_x * u
        return c_rb

    def kinematics_jacobian(self, eta: Vec4[Position, NED]) -> NDArray[np.float64]:
        """Builds J(η), which maps the body velocity ν to the NED-frame pose rates η̇.

        For the 4-DOF state η = [x, y, φ, ψ]::

            J(η) = ⎡ cos ψ  −sin ψ·cos φ  0    0   ⎤
                   ⎢ sin ψ   cos ψ·cos φ  0    0   ⎥
                   ⎢   0         0        1    0   ⎥
                   ⎣   0         0        0  cos φ ⎦

        Args:
            eta (Vec4[Position, NED]): η, the boat's pose [x, y, φ, ψ].

        Returns:
            NDArray[np.float64]: the 4x4 J(η) matrix.
        """
        c_phi = math.cos(eta.p)
        c_psi, s_psi = math.cos(eta.r), math.sin(eta.r)
        return np.array(
            [
                [c_psi, -s_psi * c_phi, 0.0, 0.0],
                [s_psi, c_psi * c_phi, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, c_phi],
            ],
            dtype=float,
        )

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
        phi, psi = self.pose.p, self.pose.r
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
