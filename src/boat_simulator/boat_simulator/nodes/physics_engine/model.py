"""This module represents the state of the boat at a given step in time."""

from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from rclpy.logging import get_logger

from boat_simulator.common.angle_conventions import (
    Heading,
    RudderAngle,
    TrimTabAngle,
)
from boat_simulator.common.constants import (
    AIR_DENSITY,
    BOAT_PROPERTIES,
    ORIENTATION_INDICES,
    WATER_DENSITY,
)
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
from boat_simulator.common.types import Mat4, Vec2, Vec3, Vec4
from boat_simulator.common.utils import ned_to_body_rotation_matrix
from boat_simulator.nodes.physics_engine.fluid_forces import MediumForceComputation
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from custom_interfaces.msg import HelperLatLon
from local_pathfinding.coord_systems import XY, meters_to_km, xy_to_latlon

_logger = get_logger(__name__)


class BoatState:
    """Represents the state of the boat at a specific point in time, including kinematic data
    in both relative and global reference frames.

    Attributes:
        `kinematics_computation` (BoatKinematics): The kinematic data for the boat in both
            the relative and global reference frames, used for computing future kinematic data,
            expressed in SI units.
    """

    def __init__(self, timestep: float, reference_latlon: HelperLatLon):
        """Initializes an instance of `BoatState`.

        Args:
            timestep (float): The time interval for calculations, expressed in seconds (s).
            reference_latlon (HelperLatLon): Geographic origin of the simulator's local XY frame,
                used to project `global_position` (meters) to lat/lon.
        """
        self.__reference_latlon = reference_latlon
        # Replace BoatKinematics as its completely all over the place
        # Switch to Equations of Motion (EOM) Class

        # TODO: Initalize boat kinematics computation
        self.__kinematics_computation = BoatKinematics(
            timestep, BOAT_PROPERTIES.mass, BOAT_PROPERTIES.inertia
        )

        # TODO: Initalize Aerodynamic Force computation (AeroDynamicsForceComputation)

        # TODO: Initalize Hydro-dynamics Force computation (HydroDynamicsForceComputation)

        # TODO: Initalize Hydro-static Force computation (HydroStaticsForceComputation)

    def step(
        self,
        glo_wind_vel: Vec2[Velocity, NED],
        glo_water_vel: Vec2[Velocity, NED],
        rudder_angle: RudderAngle,
        trim_tab_angle: TrimTabAngle,
    ) -> None:
        """Updates the boat's kinematic data based on applied forces and torques, and returns
        the updated kinematic data in both relative and global reference frames.

        Args:
            glo_wind_vel (Vec2[Velocity, NED]): The velocity of the true wind in the global
                reference frame, expressed in meters per second (m/s).
            glo_water_vel (Vec2[Velocity, NED]): The velocity of the current in the global
                reference frame, expressed in meters per second (m/s).
            rudder_angle (RudderAngle): The rudder steering angle with respect to the boat.
            trim_tab_angle (TrimTabAngle): The trim tab angle with respect to the wingsail.

        Returns:
            None: The method updates the internal state of the boat's kinematics but does not
            return any data.
        """

        """
        In functions:
        1. Clean up all the boat state values
        2. Assemble the forces and torques through the __compute_net_force_and_torque function sum!
            a. _compute_wingsail_force (Child class of MediumForceComputation -> AeroDynamicsForceComputation)
            b. _compute_hydro_force (Child class of MediumForceComputation -> HydroDynamicsForceComputation)
                i. _compute_rudder_force_and_torque
                ii. _compute_keel_force_and_torque
                iii. _compute_hull_force_and_torque
                iv. Compute M_A is added mass, C_A the added-mass Coriolis/centripetal matrix,
                    D linear damping
            c. _compute_hydrostatic_force_and_torque (Child class of MediumForceComputation -> HydroStaticsForceComputation)
        3. BoatKinematics step:
            a. Run BoatKinematics.step()
                i. Calls KinematicsFormulas function is steps as shown below
                    1. KinematicsFormulas._compute_acceleration (F = m * a :) )
                    2. KinematicsFormulas._compute_velocity (v = v_initial + a * dt)
                    3. KinematicsFormulas._compute_transformation_and_position
                    4. KinematicsFormulas._compute_transformation: η̇ = J(η)·ν
                    5. KinematicsFormulas._compute_position: (n = n_initial + η̇  * dt)
                ii. Store the acceleration, velocity and position in the attributes
        6. Wrap the angles to the angle conventions and set the actuator angles to the max angle
            for each actuator

        In equations:
        1. v_r = ν − v_c  # relative-to-water velocity (current from Fluid Sim)
        2. assemble forces at current state:
            τ_S            = wingsail   (Wingsail page,  uses apparent wind, δ_tab → α)
            τ_h, τ_r, τ_k  = hydro      (Hydrodynamics page, uses v_r, δ_r)
            g(η)           = hydrostatic restoring (Hydrostatics page)
        3. ν̇ = (M_RB + M_A)⁻¹ · [ Σ forces − C_RB·ν − C_A·v_r − D·v_r ]
        4. integrate:  ν ← ν + ν̇·dt
        5. η̇ = J(η)·ν ;  η ← η + η̇·dt
        6. wrap ψ to (−π, π]; advance actuators (δ_r, δ_tab) and environment; repeat

        """

        pass

    def __compute_net_force_and_torque(
        self,
        rel_wind_vel: Vec2[Velocity, Body],
        rel_water_vel: Vec2[Velocity, Body],
        rudder_angle_deg: float,
        sail_angle_deg: float,
    ) -> Tuple[Vec3[Force, NED], Vec3[Torque, Body]]:
        """Calculates the net force and net torque acting on the boat caused by the wind and water.

        Args:
            rel_wind_vel (Vec2[Velocity, Body]): The velocity of the true wind in the relative
                reference frame, expressed in meters per second (m/s).
            rel_water_vel (Vec2[Velocity, Body]): The velocity of the current in the relative
                reference frame, expressed in meters per second (m/s).
            rudder_angle_deg (float): The rudder angle with respect to the boat in degrees. Angle
                convention is 0° south, increases CW.
            sail_angle_deg (float): The wingsail's current orientation in degrees, using the
                convention expected by `MediumForceComputation.compute`
                (0° along +x, CCW positive).

        Returns:
            Tuple[Vec3[Force, NED], Vec3[Torque, Body]]: A tuple where the first element represents
                the net force in the global reference frame, expressed in newtons (N), and the
                second element represents the net torque, expressed in newton-meters (N•m).
        """
        # TODO: Complete the net force and torque calculations

        return (Vec3.from_xyz(0.0, 0.0, 0.0), Vec3.from_xyz(0.0, 0.0, 0.0))

    @property
    def pose(self) -> Vec4[Position, NED]:
        """Returns the boat's current position in the global reference frame,
        expressed in meters [m]."""
        return self.__kinematics_computation.pose

    @property
    def global_lat_lon_position(self) -> NDArray:
        """Returns the boat's current position projected onto geographic lat/lon coordinates,
        expressed in degrees [°].
        """
        pos_m = self.pose
        # TODO: Needs to be updated for NED
        xy_km = XY(x=meters_to_km(pos_m.x), y=meters_to_km(pos_m.y))
        latlon = xy_to_latlon(self.__reference_latlon, xy_km)
        return np.array([latlon.latitude, latlon.longitude])

    @property
    def nu(self) -> Vec4[Velocity, Body]:
        """Returns the boat's"""
        return self.__kinematics_computation.nu

    @property
    def nu_dot(self) -> Vec4[Acceleration, Body]:
        """Returns the boat's current acceleration in the Body (relative) reference frame,
        expressed in meters per second squared [m/s^2]."""
        return self.__kinematics_computation.nu_dot

    @property
    def inertia(self) -> Mat4[Inertia, Body]:
        """Returns the boat's inertia, expressed in kilogram square meters [kg•m^2]."""
        return self.__kinematics_computation.inertia

    @property
    def inertia_inverse(self) -> Mat4[InverseInertia, Body]:
        """Returns the boat's inverse inertia,
        expressed in per kilogram square meters [1/(kg•m^2)]."""
        return self.__kinematics_computation.inertia_inverse

    @property
    def boat_mass(self) -> float:
        """Returns the boat's mass, expressed in kilograms [kg]."""
        return self.__kinematics_computation.boat_mass

    @property
    def timestep(self) -> float:
        """Returns the time interval on which the boat's kinematic calculations are based,
        expressed in seconds [s]."""
        return self.__kinematics_computation.timestep

    @property
    def linear_speed(self) -> float:
        """Returns the speed on the boat, calculated as the magnitude of the velocity vector in
        the NED (global) reference frame, expressed in meters per second [m/s]."""
        return float(np.linalg.norm(x=self.nu.data, ord=2))

    @property
    def true_bearing(self) -> Heading:
        """Calculates the boat's heading in the global reference frame based on its angular
        position, using the DesiredHeading message's angle convention
        (0 degrees is straight, increasing CCW). The heading is normalized to the range [-pi, pi)
        radians (equivalently [-180, 180) degrees)."""

        orientation = ORIENTATION_INDICES["YAW"]
        yaw_rad = self.global_angular_position.data[orientation.value]
        return Heading(yaw_rad)
