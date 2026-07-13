from rclpy.logging import get_logger

from boat_simulator.common.conventions import (
    Body,
    Force,
)
from boat_simulator.common.types import Vec4
from boat_simulator.nodes.physics_engine.fluid_forces import (
    AeroDynamicsForceComputation,
    HydroDynamicsForceComputation,
    HydroStaticsForceComputation,
)
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics

_logger = get_logger(__name__)


class TotalForceComputation:
    """Assembles τ_RB, the total generalized force on the boat, from the component
    force computations in this module (van Tonder Eq. 12)::

        τ_RB = τ_hydro + τ_static + τ_aero
             = [ −M_A·ν̇_r − C_A(v_r)·v_r − D·v_r + τ_h + τ_r + τ_k ]   (HydroDynamics)
               + g(η)                                                  (HydroStatics)
               + τ_S                                                   (AeroDynamics)

    Solving the equations of motion with this force is `BoatKinematics`'s job
    (kinematics_computation.py).

    Attributes:
        hydrodynamics (HydroDynamicsForceComputation): Computes τ_hydro, the hull, rudder,
            keel, added-mass, and damping contribution.
        hydrostatics (HydroStaticsForceComputation): Computes g(η), the hydrostatic
            restoring force.
        aerodynamics (AeroDynamicsForceComputation): Computes τ_S, the wingsail
            contribution.
    """

    def __init__(self):
        self.__hydrodynamics = HydroDynamicsForceComputation()
        self.__hydrostatics = HydroStaticsForceComputation()
        self.__aerodynamics = AeroDynamicsForceComputation()

    def compute_total_force(
        self,
        boat_kinematics: BoatKinematics,
        true_wind_speed_mps: float,
        true_wind_bearing_rad: float,
        ocean_current_speed_mps: float,
        ocean_current_bearing_rad: float,
        delta_r_rad: float,
        delta_tab_rad: float,
        alpha_guess_rad: float = 0.0,
    ) -> Vec4[Force, Body]:
        """Assembles total force (τ_RB) = τ_hydro + g(η) + τ_S, the total generalized force
        on the boat.

        The current is assumed constant and irrotational, so ν̇_r = ν̇; the previous
        timestep's acceleration stands in for ν̇ in the −M_A·ν̇_r term inside τ_hydro,
        which breaks the algebraic loop of τ_RB depending on the acceleration it produces.

        Args:
            boat_kinematics (BoatKinematics): The boat kinematics (position, velocity, acceleration
                ) that contains the state at time t. The forces will help us find the state at time
                at t+1
            true_wind_speed_mps (float): V_w, the true wind speed, in meters per second.
            true_wind_bearing_rad (float): beta_w, the true wind direction as an NED
                bearing, in radians.
            ocean_current_speed (float): V_c, the ocean current speed, in meters per second.
            ocean_current_bearing_rad (float): beta_c, the current direction as an NED bearing,
                in radians.
            delta_r_rad (float): The rudder deflection, in radians.
            delta_tab_rad (float): The trim tab deflection, in radians.
            alpha_guess_rad (float): The previous timestep's solved wing angle, in radians.

        Returns:
            Vec4[Force, Body]: τ_RB = [X, Y, K, N], the total generalized force.
        """
        roll_rad, heading_rad = boat_kinematics.pose.p, boat_kinematics.pose.r

        v_r = self.__hydrodynamics.relative_velocity(
            boat_kinematics.nu, ocean_current_speed_mps, ocean_current_bearing_rad, heading_rad
        )
        hydro_force = self.__hydrodynamics.compute(
            v_r, boat_kinematics.nu_dot, roll_rad, delta_r_rad
        )
        static_force = self.__hydrostatics.compute(roll_rad)
        aero_force = self.__aerodynamics.compute(
            boat_kinematics.nu,
            roll_rad,
            true_wind_speed_mps,
            true_wind_bearing_rad,
            heading_rad,
            delta_tab_rad,
            alpha_guess_rad,
        )

        total_force = hydro_force + static_force + aero_force
        _logger.info(
            f"TotalForceComputation.compute_total_force: hydro_force={hydro_force.data} "
            f"static_force={static_force.data} aero_force={aero_force.data} "
            f"total_force={total_force.data}"
        )
        return total_force
