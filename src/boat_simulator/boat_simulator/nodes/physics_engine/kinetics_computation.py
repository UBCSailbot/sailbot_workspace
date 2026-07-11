import math
from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from rclpy.logging import get_logger

from boat_simulator.common.conventions import (
    NED,
    Acceleration,
    Body,
    Force,
    Inertia,
    Position,
    Velocity,
)
from boat_simulator.common.types import Mat4, Vec4
from boat_simulator.nodes.physics_engine.fluid_forces import (
    AeroDynamicsForceComputation,
    HydroDynamicsForceComputation,
    HydroStaticsForceComputation,
)

_logger = get_logger(__name__)


class RigidBodyDynamicsComputation:
    """Assembles the boat's 4-DOF rigid-body equations of motion from the component
    force computations in this module (van Tonder Eq. 12)::

        η̇ = J(η)·ν                     # kinematics — maps body velocity to world-frame rates
        M_RB·ν̇ + C_RB(ν)·ν = τ_RB      # kinetics  — Newton–Euler in the body frame

        τ_RB = τ_hydro + τ_static + τ_aero
             = [ −M_A·ν̇_r − C_A(v_r)·v_r − D·v_r + τ_h + τ_r + τ_k ]   (HydroDynamics)
               + g(η)                                                  (HydroStatics)
               + τ_S                                                   (AeroDynamics)

    Attributes:
        hydrodynamics (HydroDynamicsForceComputation): Computes τ_hydro, the hull, rudder,
            keel, added-mass, and damping contribution.
        hydrostatics (HydroStaticsForceComputation): Computes g(η), the hydrostatic
            restoring force.
        aerodynamics (AeroDynamicsForceComputation): Computes τ_S, the wingsail
            contribution.
        m_rb (Mat4[Inertia, Body]): M_RB, the rigid-body mass-inertia matrix.
    """

    def __init__(
        self,
        hydrodynamics: HydroDynamicsForceComputation,
        hydrostatics: HydroStaticsForceComputation,
        aerodynamics: AeroDynamicsForceComputation,
        m_rb: Mat4[Inertia, Body],
    ):
        self.__hydrodynamics = hydrodynamics
        self.__hydrostatics = hydrostatics
        self.__aerodynamics = aerodynamics
        self.__m_rb = m_rb
        # M_RB is constant, so invert it once here rather than every timestep.
        self.__m_rb_inv = np.linalg.inv(m_rb.data)

    def compute_total_force(
        self,
        eta: Vec4[Position, NED],
        nu: Vec4[Velocity, Body],
        nu_dot_prev: Vec4[Acceleration, Body],
        true_wind_speed: float,
        true_wind_bearing_rad: float,
        current_speed: float,
        current_bearing_rad: float,
        delta_r_rad: float,
        delta_tab_rad: float,
        alpha_guess_rad: float = 0.0,
    ) -> Vec4[Force, Body]:
        """Assembles τ_RB = τ_hydro + g(η) + τ_S, the total generalized force on the boat.

        The current is assumed constant and irrotational, so ν̇_r = ν̇; the previous
        timestep's acceleration stands in for ν̇ in the −M_A·ν̇_r term inside τ_hydro,
        which breaks the algebraic loop of τ_RB depending on the acceleration it produces.

        Args:
            eta (Vec4[Position, NED]): η, the boat's pose [x, y, φ, ψ].
            nu (Vec4[Velocity, Body]): ν, the boat's generalized velocity [u, v, p, r].
            nu_dot_prev (Vec4[Acceleration, Body]): The previous timestep's solved
                acceleration ν̇, used for the added-mass term.
            true_wind_speed (float): V_w, the true wind speed, in meters per second.
            true_wind_bearing_rad (float): beta_w, the true wind direction as an NED
                bearing, in radians.
            current_speed (float): V_c, the ocean current speed, in meters per second.
            current_bearing_rad (float): beta_c, the current direction as an NED bearing,
                in radians.
            delta_r_rad (float): The rudder deflection, in radians.
            delta_tab_rad (float): The trim tab deflection, in radians.
            alpha_guess_rad (float): The previous timestep's solved wing angle, in radians.

        Returns:
            Vec4[Force, Body]: τ_RB = [X, Y, K, N], the total generalized force.
        """
        roll_rad, heading_rad = eta.p, eta.r

        v_r = self.__hydrodynamics.relative_velocity(
            nu, current_speed, current_bearing_rad, heading_rad
        )
        tau_hydro = self.__hydrodynamics.compute(v_r, nu_dot_prev, roll_rad, delta_r_rad)
        tau_static = self.__hydrostatics.compute(roll_rad)
        tau_aero = self.__aerodynamics.compute(
            nu,
            roll_rad,
            true_wind_speed,
            true_wind_bearing_rad,
            heading_rad,
            delta_tab_rad,
            alpha_guess_rad,
        )

        tau_rb = tau_hydro + tau_static + tau_aero
        _logger.info(
            f"RigidBodyDynamics.compute_total_force: tau_hydro={tau_hydro.data} "
            f"tau_static={tau_static.data} tau_aero={tau_aero.data} tau_rb={tau_rb.data}"
        )
        return tau_rb

    def compute_acceleration(
        self, nu: Vec4[Velocity, Body], tau_rb: Vec4[Force, Body]
    ) -> Vec4[Acceleration, Body]:
        """Solves the kinetics equation for the body-frame acceleration:
        ν̇ = M_RB⁻¹·(τ_RB − C_RB(ν)·ν).

        Args:
            nu (Vec4[Velocity, Body]): ν, the boat's generalized velocity [u, v, p, r].
            tau_rb (Vec4[Force, Body]): τ_RB, the total generalized force.

        Returns:
            Vec4[Acceleration, Body]: ν̇ = [u̇, v̇, ṗ, ṙ], the generalized acceleration.
        """
        coriolis_term = self.rigid_body_coriolis_matrix(nu) @ nu.data
        return Vec4(self.__m_rb_inv @ (tau_rb.data - coriolis_term))

    def compute_kinematics(
        self, eta: Vec4[Position, NED], nu: Vec4[Velocity, Body]
    ) -> Vec4[Velocity, NED]:
        """Evaluates the kinematics equation η̇ = J(η)·ν.

        Args:
            eta (Vec4[Position, NED]): η, the boat's pose [x, y, φ, ψ].
            nu (Vec4[Velocity, Body]): ν, the boat's generalized velocity [u, v, p, r].

        Returns:
            Vec4[Velocity, NED]: η̇ = [ẋ, ẏ, φ̇, ψ̇], the world-frame pose rates.
        """
        return Vec4(self.kinematics_jacobian(eta) @ nu.data)

    def compute(
        self,
        eta: Vec4[Position, NED],
        nu: Vec4[Velocity, Body],
        nu_dot_prev: Vec4[Acceleration, Body],
        true_wind_speed: float,
        true_wind_bearing_rad: float,
        current_speed: float,
        current_bearing_rad: float,
        delta_r_rad: float,
        delta_tab_rad: float,
        alpha_guess_rad: float = 0.0,
    ) -> Tuple[Vec4[Velocity, NED], Vec4[Acceleration, Body]]:
        """Evaluates one step of the full rigid-body model, returning the state derivatives
        (η̇, ν̇) for the integrator.

        Args:
            eta (Vec4[Position, NED]): η, the boat's pose [x, y, φ, ψ].
            nu (Vec4[Velocity, Body]): ν, the boat's generalized velocity [u, v, p, r].
            nu_dot_prev (Vec4[Acceleration, Body]): The previous timestep's solved
                acceleration ν̇, used for the added-mass term.
            true_wind_speed (float): V_w, the true wind speed, in meters per second.
            true_wind_bearing_rad (float): beta_w, the true wind direction as an NED
                bearing, in radians.
            current_speed (float): V_c, the ocean current speed, in meters per second.
            current_bearing_rad (float): beta_c, the current direction as an NED bearing,
                in radians.
            delta_r_rad (float): The rudder deflection, in radians.
            delta_tab_rad (float): The trim tab deflection, in radians.
            alpha_guess_rad (float): The previous timestep's solved wing angle, in radians.

        Returns:
            Tuple[Vec4[Velocity, NED], Vec4[Acceleration, Body]]: (η̇, ν̇), the pose rates
                and the generalized acceleration.
        """
        tau_rb = self.compute_total_force(
            eta,
            nu,
            nu_dot_prev,
            true_wind_speed,
            true_wind_bearing_rad,
            current_speed,
            current_bearing_rad,
            delta_r_rad,
            delta_tab_rad,
            alpha_guess_rad,
        )
        nu_dot = self.compute_acceleration(nu, tau_rb)
        eta_dot = self.compute_kinematics(eta, nu)
        return eta_dot, nu_dot
