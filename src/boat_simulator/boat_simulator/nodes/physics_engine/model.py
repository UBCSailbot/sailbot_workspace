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
from boat_simulator.common.types import Mat3, Vec2, Vec3
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
        glo_vel_2d = self.global_velocity.data[:2]
        orientation = self.global_angular_position
        ned_to_body = ned_to_body_rotation_matrix(
            roll_rad=orientation.y,
            pitch_rad=orientation.x,
            yaw_rad=orientation.z,
        )[:2, :2]
        rel_wind_vel: Vec2[Velocity, Body] = Vec2(ned_to_body @ (glo_wind_vel.data - glo_vel_2d))
        rel_water_vel: Vec2[Velocity, Body] = Vec2(ned_to_body @ (glo_water_vel.data - glo_vel_2d))

        rudder_angle_deg = rudder_angle.degrees

        _logger.debug(
            f"BS | step inputs: rel_wind_vel={rel_wind_vel} rudder_angle={rudder_angle_deg:.2f} "
            + f"trim_tab={trim_tab_angle.degrees:.2f}"
        )

        sail_angle_deg = trim_tab_angle.degrees

        glo_net_force, net_torque = self.__compute_net_force_and_torque(
            rel_wind_vel, rel_water_vel, rudder_angle_deg, sail_angle_deg
        )

        # Guard the integration boundary: a single non-finite force/torque (e.g. from an upstream
        # numerical issue) would be integrated into position/velocity and, because NaN/Inf are
        # absorbing, corrupt every subsequent step irrecoverably. Drop the bad update instead.
        if not (np.all(np.isfinite(glo_net_force.data)) and np.all(np.isfinite(net_torque.data))):
            _logger.error(
                f"BS | non-finite force/torque, skipping step: "
                f"net_force={glo_net_force} net_torque={net_torque}"
            )
            return

        self.__kinematics_computation.step(glo_net_force, net_torque)

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

        # sail_lift, sail_drag = self.__sail_force_computation.compute(rel_wind_vel, sail_angle_deg)
        # rudder_lift, rudder_drag = self.__rudder_force_computation.compute(
        #     rel_water_vel, rudder_angle_deg
        # )

        # Hull drag is computed in NED from the global velocity. Rotate the body-frame sail and
        # rudder forces into NED before adding it so the force sum never mixes reference frames.

        # glo_vel_2d = self.global_velocity.data[:2]
        # hull_drag = -self.hull_drag_factor * np.linalg.norm(glo_vel_2d) * glo_vel_2d
        # body_force = sail_lift.data + sail_drag.data + rudder_lift.data + rudder_drag.data
        # orientation = self.global_angular_position
        # body_to_ned = ned_to_body_rotation_matrix(
        #     roll_rad=orientation.y,
        #     pitch_rad=orientation.x,
        #     yaw_rad=orientation.z,
        # ).T[:2, :2]

        # net_force: Vec3[Force, NED] = Vec3(body_to_ned @ body_force + hull_drag)  # ?
        # tau_z = self.sail_dist * (sail_lift.y + sail_drag.y) - self.rudder_dist * (
        #     rudder_lift.y + rudder_drag.y
        # )

        # (net_force, tau_z_vector) — currently stubbed to zero while the dynamics are validated.

        return (Vec3.from_xyz(0.0, 0.0, 0.0), Vec3.from_xyz(0.0, 0.0, 0.0))

    @property
    def global_position(self) -> Vec3[Position, NED]:
        """Returns the boat's current position in the global reference frame,
        expressed in meters [m]."""
        return self.__kinematics_computation.global_data.linear_position

    @property
    def global_lat_lon_position(self) -> NDArray:
        """Returns the boat's current position projected onto geographic lat/lon coordinates,
        expressed in degrees [°].
        """
        pos_m = self.global_position
        xy_km = XY(x=meters_to_km(pos_m.x), y=meters_to_km(pos_m.y))
        latlon = xy_to_latlon(self.__reference_latlon, xy_km)
        return np.array([latlon.latitude, latlon.longitude])

    @property
    def global_velocity(self) -> Vec3[Velocity, NED]:
        """Returns the boat's current velocity in the NED (global) reference frame,
        expressed in meters per second [m/s]."""
        return self.__kinematics_computation.global_data.linear_velocity

    @property
    def global_acceleration(self) -> Vec3[Acceleration, NED]:
        """Returns the boat's current acceleration in the NEd (global) reference frame,
        expressed in meters per second squared [m/s^2]."""
        return self.__kinematics_computation.global_data.linear_acceleration

    @property
    def global_angular_position(self) -> Vec3[Position, NED]:
        """Returns global angular position in the NED (global) reference frame,
        expressed in radians"""
        return self.__kinematics_computation.global_data.angular_position

    @property
    def relative_velocity(self) -> Vec3[Velocity, Body]:
        """Returns the boat's current velocity in the Body (relative) reference frame,
        expressed in meters per second [m/s]."""
        orientation = self.global_angular_position
        glo = self.__kinematics_computation.global_data.linear_velocity.data
        ned_to_body = ned_to_body_rotation_matrix(
            roll_rad=orientation.y,
            pitch_rad=orientation.x,
            yaw_rad=orientation.z,
        )
        return Vec3(ned_to_body @ glo)

    @property
    def relative_acceleration(self) -> Vec3[Acceleration, Body]:
        """Returns the boat's current acceleration in the Body (relative) reference frame,
        expressed in meters per second squared [m/s^2]."""
        return self.__kinematics_computation.relative_data.linear_acceleration

    @property
    def angular_position(self) -> Vec3[Position, Body]:
        """Returns the boat's current angular position along the yaw axis in the NED (global)
        reference frame, expressed in radians [rad]."""
        return self.__kinematics_computation.relative_data.angular_position

    @property
    def angular_velocity(self) -> Vec3[Velocity, Body]:
        """Returns the boat's current angular velocity along the yaw axis in the NED (global)
        reference frame, expressed in radians per second [rad/s]."""
        return self.__kinematics_computation.relative_data.angular_velocity

    @property
    def angular_acceleration(self) -> Vec3[Acceleration, Body]:
        """Returns the boat's current angular acceleration along the yaw axis in the NED
        (global) reference frame, expressed in radians per second squared [rad/s^2]."""
        return self.__kinematics_computation.relative_data.angular_acceleration

    @property
    def inertia(self) -> Mat3[Inertia, Body]:
        """Returns the boat's inertia, expressed in kilogram square meters [kg•m^2]."""
        return self.__kinematics_computation.inertia

    @property
    def inertia_inverse(self) -> Mat3[InverseInertia, Body]:
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
    def speed(self) -> float:
        """Returns the speed on the boat, calculated as the magnitude of the velocity vector in
        the MED (global) reference frame, expressed in meters per second [m/s]."""
        return float(np.linalg.norm(x=self.global_velocity.data, ord=2))

    @property
    def true_bearing(self) -> Heading:
        """Calculates the boat's heading in the global reference frame based on its angular
        position, using the DesiredHeading message's angle convention
        (0 degrees is straight, increasing CCW). The heading is normalized to the range [-pi, pi)
        radians (equivalently [-180, 180) degrees)."""

        orientation = ORIENTATION_INDICES["YAW"]
        yaw_rad = self.global_angular_position.data[orientation.value]
        return Heading(yaw_rad)
