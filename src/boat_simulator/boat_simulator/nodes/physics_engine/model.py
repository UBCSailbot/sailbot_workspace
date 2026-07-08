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
from boat_simulator.common.types import Mat4, Vec2, Vec4
from boat_simulator.common.utils import ned_to_body_rotation_matrix
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from custom_interfaces.msg import HelperLatLon
from local_pathfinding.coord_systems import XY, meters_to_km, xy_to_latlon

_logger = get_logger(__name__)


class BoatState:
    """Represents the state of the boat at a specific point in time: its pose, body velocity,
    and body acceleration, advanced each timestep by the net force and torque acting on the
    boat from the wind, water, rudder, and sail.
    """

    def __init__(self, timestep: float, reference_latlon: HelperLatLon):
        """Initializes an instance of `BoatState`.

        Args:
            timestep (float): The time interval for calculations, expressed in seconds (s).
            reference_latlon (HelperLatLon): Geographic origin of the simulator's local XY frame,
                used to project `global_position` (meters) to lat/lon.
        """
        self.__reference_latlon = reference_latlon
        self.__kinematics_computation = BoatKinematics(timestep)

    def step(
        self,
        glo_wind_vel: Vec2[Velocity, NED],
        glo_water_vel: Vec2[Velocity, NED],
        rudder_angle: RudderAngle,
        trim_tab_angle: TrimTabAngle,
    ) -> None:
        """Advances the boat's kinematic state by one timestep.

        Converts the true wind and current into body-frame relative velocities, assembles the
        net force and torque acting on the boat (see `__compute_net_force_and_torque`), and
        integrates the equations of motion via `BoatKinematics.step`.

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
        boat_vel: Vec2[Velocity, Body] = Vec2[Velocity, Body].from_xy(self.nu.x, self.nu.y)
        # Apparent wind is the true wind felt onboard, net of the boat's own motion; relative
        # water velocity is the boat's motion through the current (design doc: v_r = nu - v_c).
        rel_wind_vel = self.__ned_to_body_velocity(glo_wind_vel) - boat_vel
        rel_water_vel = boat_vel - self.__ned_to_body_velocity(glo_water_vel)

        net_force, net_torque = self.__compute_net_force_and_torque(
            rel_wind_vel, rel_water_vel, rudder_angle.degrees, trim_tab_angle.degrees
        )

        self.__kinematics_computation.step(net_force, net_torque)

    def __ned_to_body_velocity(self, glo_vel: Vec2[Velocity, NED]) -> Vec2[Velocity, Body]:
        """Rotates a NED-frame velocity into the body frame using the boat's current roll and
        heading; pitch is always 0 since it is not modeled in the 4-DOF state.
        """
        rotation = ned_to_body_rotation_matrix(
            roll_rad=self.pose.z, pitch_rad=0.0, yaw_rad=self.pose.w
        )
        rotated = rotation @ np.array([glo_vel.x, glo_vel.y, 0.0])
        return Vec2[Velocity, Body].from_xy(rotated[0], rotated[1])

    def __compute_net_force_and_torque(
        self,
        rel_wind_vel: Vec2[Velocity, Body],
        rel_water_vel: Vec2[Velocity, Body],
        rudder_angle_deg: float,
        sail_angle_deg: float,
    ) -> Tuple[Vec4[Force, Body], Vec4[Torque, Body]]:
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
            Tuple[Vec4[Force, Body], Vec4[Torque, Body]]: A tuple of the net generalized force
                [X, Y, K, N] and net generalized torque, both expressed in the body frame and
                ready to be passed to `BoatKinematics.step`.
        """
        # TODO: Not implemented yet. Assemble the wingsail (aerodynamic), hydrodynamic
        # (rudder/keel/hull, including added mass M_A, Coriolis C_A, and damping D), and
        # hydrostatic forces/moments described in the physics engine design doc. This will need
        # AIR_DENSITY/WATER_DENSITY (boat_simulator.common.constants) and MediumForceComputation
        # subclasses (boat_simulator.nodes.physics_engine.fluid_forces) for the aero/hydro force
        # laws.
        return (
            Vec4[Force, Body].from_xypr(0.0, 0.0, 0.0, 0.0),
            Vec4[Torque, Body].from_xypr(0.0, 0.0, 0.0, 0.0),
        )

    @property
    def pose(self) -> Vec4[Position, NED]:
        """Returns the boat's current pose [N, E, phi, psi] in the global (NED) reference frame:
        north/east position in meters [m], roll/heading in radians [rad]."""
        return self.__kinematics_computation.pose

    @property
    def global_lat_lon_position(self) -> NDArray:
        """Returns the boat's current position projected onto geographic lat/lon coordinates,
        expressed in degrees [°].
        """
        pos_m = self.pose
        xy_km = XY(x=meters_to_km(pos_m.x), y=meters_to_km(pos_m.y))
        latlon = xy_to_latlon(self.__reference_latlon, xy_km)
        return np.array([latlon.latitude, latlon.longitude])

    @property
    def nu(self) -> Vec4[Velocity, Body]:
        """Returns the boat's current velocity [u, v, p, r] in the Body (relative) reference
        frame: surge/sway in meters per second [m/s], roll/yaw rate in radians per second
        [rad/s]."""
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
        """Returns the boat's speed over ground, i.e. the magnitude of its surge/sway velocity
        (rotation-invariant, so this equals the NED-frame speed too), expressed in meters per
        second [m/s]."""
        return float(np.linalg.norm(x=self.nu.data[:2], ord=2))

    @property
    def true_bearing(self) -> Heading:
        """Returns the boat's current heading (yaw), normalized to the range [-pi, pi) radians
        (equivalently [-180, 180) degrees), in the simulator's own angle convention."""
        return Heading(self.pose.w)
