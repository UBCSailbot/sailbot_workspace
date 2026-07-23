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
             = [ −C_A(v_r)·v_r − D·v_r + τ_h + τ_r + τ_k ]   (HydroDynamics)
               + g(η)                                        (HydroStatics)
               + τ_S                                          (AeroDynamics)

    The added-mass inertia term −M_A·ν̇ is not a force here; M_A is folded into the
    mass matrix on the left-hand side of the equations of motion, which `BoatKinematics`
    (kinematics_computation.py) solves as ν̇ = (M_RB + M_A)⁻¹·(τ_RB − C_RB·ν). Applying
    it as an explicit force would require the previous timestep's acceleration, which is
    numerically unstable when M_A is comparable to M_RB.

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
        on the boat. Note that if the forces themselves will have a negative sign if drag > lift.
        We simply add these forces here.

        Args:
            boat_kinematics (BoatKinematics): The boat kinematics (position, velocity, acceleration
                ) that contains the state at time t. The forces will help us find the state at time
                at t+1
            true_wind_speed_mps (float): V_w, the true wind speed, in meters per second.
            true_wind_bearing_rad (float): beta_w, the NED bearing of the wind's velocity
                vector (flow-toward convention), in radians.
            ocean_current_speed (float): V_c, the ocean current speed, in meters per second.
            ocean_current_bearing_rad (float): beta_c, the NED bearing of the current's
                velocity vector (flow-toward convention), in radians.
            delta_r_rad (float): The rudder deflection, in radians.
            delta_tab_rad (float): The trim tab deflection, in radians.
            alpha_guess_rad (float): The previous timestep's solved wing angle, in radians.

        Returns:
            Vec4[Force, Body]: τ_RB = [X, Y, K, N], the total generalized force.
        """

        roll_rad, heading_rad = boat_kinematics.pose.p, boat_kinematics.pose.r

        # Calculate the relative velocities and angles
        rel_vel_water_mps = self.__hydrodynamics.relative_velocity(
            boat_kinematics.nu, ocean_current_speed_mps, ocean_current_bearing_rad, heading_rad
        )
        aw_vel_mps, aw_angle_rad = self.__aerodynamics.apparent_wind(
            boat_kinematics.nu, true_wind_speed_mps, true_wind_bearing_rad, heading_rad
        )
        wing_angle_of_attack_rad = self.__aerodynamics.solve_wing_angle(
            aw_vel_mps, delta_tab_rad, alpha_guess_rad
        )

        # Compute the class of forces
        hydro_force = self.__hydrodynamics.compute(rel_vel_water_mps, roll_rad, delta_r_rad)
        static_force = self.__hydrostatics.compute(roll_rad)
        aero_force = self.__aerodynamics.compute(
            roll_rad,
            aw_vel_mps,
            aw_angle_rad,
            wing_angle_of_attack_rad,
        )

        # Sum the forces
        total_force = hydro_force + static_force + aero_force

        _logger.debug(
            f"TotalForceComputation.compute_total_force: hydro_force={hydro_force.data} "
            f"static_force={static_force.data} aero_force={aero_force.data} "
            f"total_force={total_force.data}"
        )

        return total_force
