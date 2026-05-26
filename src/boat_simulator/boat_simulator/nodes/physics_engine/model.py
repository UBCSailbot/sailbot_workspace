"""This module represents the state of the boat at a given step in time."""

from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from rclpy.logging import get_logger

_logger = get_logger(__name__)

from boat_simulator.common.constants import (
    AIR_DENSITY,
    BOAT_PROPERTIES,
    ORIENTATION_INDICES,
    WATER_DENSITY,
)
from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180, rad_to_degrees
from boat_simulator.nodes.physics_engine.fluid_forces import MediumForceComputation
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from custom_interfaces.msg import HelperLatLon
from local_pathfinding.coord_systems import XY, meters_to_km, xy_to_latlon


class BoatState:
    """Represents the state of the boat at a specific point in time, including kinematic data
    in both relative and global reference frames.

    Attributes:
        `kinematics_computation` (BoatKinematics): The kinematic data for the boat in both
            the relative and global reference frames, used for computing future kinematic data,
            expressed in SI units.
    """

    def __init__(self, timestep: Scalar, reference_latlon: HelperLatLon):
        """Initializes an instance of `BoatState`.

        Args:
            timestep (Scalar): The time interval for calculations, expressed in seconds (s).
            reference_latlon (HelperLatLon): Geographic origin of the simulator's local XY frame,
                used to project `global_position` (meters) to lat/lon.
        """
        self.__reference_latlon = reference_latlon
        self.__kinematics_computation = BoatKinematics(
            timestep, BOAT_PROPERTIES.mass, BOAT_PROPERTIES.inertia
        )
        self.__sail_force_computation = MediumForceComputation(
            BOAT_PROPERTIES.sail_lift_coeffs,
            BOAT_PROPERTIES.sail_drag_coeffs,
            BOAT_PROPERTIES.sail_areas,
            AIR_DENSITY,
        )
        self.__rudder_force_computation = MediumForceComputation(
            BOAT_PROPERTIES.rudder_lift_coeffs,
            BOAT_PROPERTIES.rudder_drag_coeffs,
            BOAT_PROPERTIES.rudder_areas,
            WATER_DENSITY,
        )
        self.hull_drag_factor = BOAT_PROPERTIES.hull_drag_factor
        self.sail_dist = BOAT_PROPERTIES.sail_dist
        self.rudder_dist = BOAT_PROPERTIES.rudder_dist

    def step(
        self,
        glo_wind_vel: NDArray,
        glo_water_vel: NDArray,
        rudder_angle_deg: Scalar,
        trim_tab_angle: Scalar,
    ) -> None:
        """Updates the boat's kinematic data based on applied forces and torques, and returns
        the updated kinematic data in both relative and global reference frames.

        Args:
            glo_wind_vel (NDArray): The velocity of the true wind in the global reference frame,
                expressed in meters per second (m/s).
            glo_water_vel (NDArray): The velocity of the current in the global reference frame,
                expressed in meters per second (m/s).
            rudder_angle_deg (float): The rudder angle with respect to the boat in degrees. Angle
                convention is 0 degrees is straight, increasing CCW.
            trim_tab_angle (float): The trim tab angle with respect to the wingsail in degrees.
                Angle convention is 0 degrees is straight, increasing CCW.

        Returns:
            None: The method updates the internal state of the boat's kinematics but does not
            return any data.
        """
        rel_wind_vel = glo_wind_vel[:2] - self.global_velocity[:2]
        rel_water_vel = glo_water_vel[:2] - self.global_velocity[:2]  # slice into 2d vector

        _logger.debug(
            f"step inputs: rel_wind_vel={rel_wind_vel} rel_water_vel={rel_water_vel} rudder_angle={rudder_angle_deg:.2f} trim_tab={trim_tab_angle:.2f}"
        )

        rel_net_force, net_torque = self.__compute_net_force_and_torque(
            rel_wind_vel, rel_water_vel, rudder_angle_deg, trim_tab_angle
        )

        _logger.debug(f"step outputs: rel_net_force={rel_net_force} net_torque={net_torque}")

        self.__kinematics_computation.step(rel_net_force, net_torque)

    def __compute_net_force_and_torque(
        self,
        rel_wind_vel: NDArray,
        rel_water_vel: NDArray,
        rudder_angle_deg: Scalar,
        trim_tab_angle: Scalar,
    ) -> Tuple[NDArray, NDArray]:
        """Calculates the net force and net torque acting on the boat caused by the wind and water.

        Args:
            rel_wind_vel (NDArray): The velocity of the true wind in the relative reference frame,
                expressed in meters per second (m/s).
            rel_water_vel (NDArray): The velocity of the current in the relative reference frame,
                expressed in meters per second (m/s).
            rudder_angle_deg (float): The rudder angle with respect to the boat in degrees. Angle
                convention is 0 degrees is straight, increasing CCW.
            trim_tab_angle (float): The trim tab angle with respect to the wingsail in degrees.
                Angle convention is 0 degrees is straight, increasing CCW.

        Returns:
            Tuple[NDArray, NDArray]: A tuple where the first element represents the net force in
                the relative reference frame, expressed in newtons (N), and the second element
                represents the net torque, expressed in newton-meters (N•m).
        """

        sail_lift, sail_drag = self.__sail_force_computation.compute(rel_wind_vel, trim_tab_angle)
        rudder_lift, rudder_drag = self.__rudder_force_computation.compute(
            rel_water_vel, rudder_angle_deg
        )
        rel_vel_2d = self.relative_velocity[:2]
        hull_drag = -self.hull_drag_factor * np.linalg.norm(rel_vel_2d) * rel_vel_2d
        net_force = sail_lift + sail_drag + rudder_lift + rudder_drag + hull_drag
        tau_z = self.sail_dist * (sail_lift[1] + sail_drag[1]) - self.rudder_dist * (
            rudder_lift[1] + rudder_drag[1]
        )

        tau_z_vector = np.array([0.0, 0.0, tau_z])
        net_force = np.array(
            [net_force[0], net_force[1], 0.0]
        )  # slice into 2d vector and add zero z-comp

        _logger.info(f"Computed forces: net_force={net_force}, tau_z_vector={tau_z_vector}")

        return (net_force, tau_z_vector)

    @property
    def global_position(self) -> NDArray:
        """Returns the boat's current position in the global reference frame,
        expressed in meters [m]."""
        return self.__kinematics_computation.global_data.linear_position

    @property
    def global_lat_lon_position(self) -> NDArray:
        """Returns the boat's current position projected onto geographic lat/lon coordinates,
        expressed in degrees [°].
        """
        pos_m = self.global_position
        xy_km = XY(x=meters_to_km(float(pos_m.item(0))), y=meters_to_km(float(pos_m.item(1))))
        latlon = xy_to_latlon(self.__reference_latlon, xy_km)
        return np.array([latlon.latitude, latlon.longitude])

    @property
    def global_velocity(self) -> NDArray:
        """Returns the boat's current velocity in the global reference frame,
        expressed in meters per second [m/s]."""
        return self.__kinematics_computation.global_data.linear_velocity

    @property
    def global_acceleration(self) -> NDArray:
        """Returns the boat's current acceleration in the global reference frame,
        expressed in meters per second squared [m/s^2]."""
        return self.__kinematics_computation.global_data.linear_acceleration

    @property
    def relative_velocity(self) -> NDArray:
        """Returns the boat's current velocity in the relative reference frame,
        expressed in meters per second [m/s]."""
        return self.__kinematics_computation.relative_data.linear_velocity

    @property
    def relative_acceleration(self) -> NDArray:
        """Returns the boat's current acceleration in the relative reference frame,
        expressed in meters per second squared [m/s^2]."""
        return self.__kinematics_computation.relative_data.linear_acceleration

    @property
    def angular_position(self) -> NDArray:
        """Returns the boat's current angular position along the yaw axis in the global reference
        frame, expressed in radians [rad]."""
        return self.__kinematics_computation.relative_data.angular_position

    @property
    def angular_velocity(self) -> NDArray:
        """Returns the boat's current angular velocity along the yaw axis in the global reference
        frame, expressed in radians per second [rad/s]."""
        return self.__kinematics_computation.relative_data.angular_velocity

    @property
    def angular_acceleration(self) -> NDArray:
        """Returns the boat's current angular acceleration along the yaw axis in the
        global reference frame, expressed in radians per second squared [rad/s^2]."""
        return self.__kinematics_computation.relative_data.angular_acceleration

    @property
    def inertia(self) -> NDArray:
        """Returns the boat's inertia, expressed in kilogram square meters [kg•m^2]."""
        return self.__kinematics_computation.inertia

    @property
    def inertia_inverse(self) -> NDArray:
        """Returns the boat's inverse inertia,
        expressed in per kilogram square meters [1/(kg•m^2)]."""
        return self.__kinematics_computation.inertia_inverse

    @property
    def boat_mass(self) -> Scalar:
        """Returns the boat's mass, expressed in kilograms [kg]."""
        return self.__kinematics_computation.boat_mass

    @property
    def timestep(self) -> Scalar:
        """Returns the time interval on which the boat's kinematic calculations are based,
        expressed in seconds [s]."""
        return self.__kinematics_computation.timestep

    @property
    def speed(self) -> Scalar:
        """Returns the speed on the boat, calculated as the magnitude of the velocity vector in
        the global reference frame, expressed in meters per second [m/s]."""
        return float(np.linalg.norm(x=self.global_velocity, ord=2))

    @property
    def true_bearing(self) -> Scalar:
        """Calculates the boat's heading in the global reference frame based on its angular
        position, using the DesiredHeading message's angle convention
        (0 degrees is straight, increasing CCW). The heading is normalized to the range [-180, 180]
        degrees."""
        yaw_rad = self.angular_position[ORIENTATION_INDICES.YAW.value]
        return bound_to_180(rad_to_degrees(yaw_rad))
