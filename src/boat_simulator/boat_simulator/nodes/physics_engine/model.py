"""This module represents the state of the boat at a given step in time."""

import math

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
    Velocity,
)
from boat_simulator.common.types import Mat4, Vec2, Vec4
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinetics_computation import (
    TotalForceComputation,
)
from custom_interfaces.msg import HelperLatLon
from local_pathfinding.coord_systems import XY, meters_to_km, xy_to_latlon

_logger = get_logger(__name__)


class BoatState:
    """Represents the state of the boat at a specific point in time: its pose, body velocity,
    and body acceleration, advanced each timestep by the net force and torque acting on the
    boat from the wind, water, rudder, and sail.
    """

    def __init__(self, timestep: float, reference_latlon: HelperLatLon, substeps: int = 1):
        """Initializes an instance of `BoatState`.

        Args:
            timestep (float): The integration time interval, expressed in seconds (s). Kept
                small enough for the explicit Euler integration in `BoatKinematics` to stay
                numerically stable under the quadratic drag terms.
            reference_latlon (HelperLatLon): Geographic origin of the simulator's local XY frame,
                used to project `global_position` (meters) to lat/lon.
            substeps (int): The number of `timestep`-sized integration steps to run per call to
                `step()`, so callers can advance the simulation by the publish period
                while integrating at a smaller, stable timestep internally.
        """
        self.__reference_latlon = reference_latlon
        self.__kinematics_computation = BoatKinematics(timestep)
        self.__kinetics_computation = TotalForceComputation()
        self.__substeps = substeps

    def step(
        self,
        true_wind_vel: Vec2[Velocity, NED],
        glo_water_current_vel: Vec2[Velocity, NED],
        rudder_angle: RudderAngle,
        trim_tab_angle: TrimTabAngle,
    ) -> None:
        """Advances the boat's kinematic state by one timestep.

        Converts the true wind and current velocity vectors into the speed and NED-bearing
        form to compute the total force, then repeats, for `substeps` iterations,
        creating the total generalized force on the boat from the current state and integrating
        the equations of Motions (BoatKinematics). Forces are recomputed each substep.

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

        # Both bearings follow the simulator's flow-toward convention: the NED bearing of
        # the fluid's velocity vector (the direction it flows TOWARD, not the direction it
        # comes from). Consumers in fluid_forces.py assume this for wind and current alike.
        true_wind_speed_mps = math.hypot(true_wind_vel.x, true_wind_vel.y)
        true_wind_bearing_rad = math.atan2(true_wind_vel.y, true_wind_vel.x)
        ocean_current_speed_mps = math.hypot(glo_water_current_vel.x, glo_water_current_vel.y)
        ocean_current_bearing_rad = math.atan2(glo_water_current_vel.y, glo_water_current_vel.x)

        for _ in range(self.__substeps):
            net_force: Vec4[Force, Body] = self.__kinetics_computation.compute_total_force(
                boat_kinematics=self.__kinematics_computation,
                true_wind_speed_mps=true_wind_speed_mps,
                true_wind_bearing_rad=true_wind_bearing_rad,
                ocean_current_speed_mps=ocean_current_speed_mps,
                ocean_current_bearing_rad=ocean_current_bearing_rad,
                delta_r_rad=rudder_angle.radians,
                delta_tab_rad=trim_tab_angle.radians,
            )
            self.__kinematics_computation.step(self.nu, net_force)

        _logger.info(
            f"timestep {self.timestep}\n"
            f"  pose      x={self.pose.x:8.3f} m     y={self.pose.y:8.3f} m     "
            f"roll={math.degrees(self.pose.p):7.2f} deg   "
            f"yaw={math.degrees(self.pose.r):7.2f} deg\n"
            f"  velocity  u={self.nu.x:8.3f} m/s   v={self.nu.y:8.3f} m/s   "
            f"roll_rate={math.degrees(self.nu.p):7.2f} deg/s   "
            f"yaw_rate={math.degrees(self.nu.r):7.2f} deg/s\n"
            f"  accel     u_dot={self.nu_dot.x:8.3f} m/s^2   v_dot={self.nu_dot.y:8.3f} m/s^2   "
            f"roll_acc={math.degrees(self.nu_dot.p):7.2f} deg/s^2   "
            f"yaw_acc={math.degrees(self.nu_dot.r):7.2f} deg/s^2"
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
        return Heading(self.pose.r)
