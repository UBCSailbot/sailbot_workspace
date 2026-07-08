"""Formulas for estimating the next position, velocity, and acceleration."""

from boat_simulator.common.conventions import (
    Acceleration,
    Force,
    InverseInertia,
    Position,
    Torque,
    Velocity,
)
from boat_simulator.common.types import Frame, Mat4, Vec4


class KinematicsFormulas:
    """Contains formulas for calculating the boat's next position, velocity, and acceleration based
    on previous kinematic data.

    Every formula operates on and returns :class:`Vec4` values so the reference frame and
    physical quantity carried by each vector is preserved through the integration. Arithmetic is
    performed on the underlying ``.data`` arrays because the integration deliberately combines
    vectors of different quantities (e.g. position += velocity·dt), which the quantity-tagged
    ``Vec4`` operators intentionally forbid.
    """

    @staticmethod
    def next_position(
        pos: Vec4[Position, Frame],
        vel: Vec4[Velocity, Frame],
        timestep: float,
    ) -> Vec4[Position, Frame]:
        """Calculates the boat's next position based on previous data and time step. Can be used
        for both linear and angular positions.

        Args:
            pos (Vec4[Position, Frame]): The last recorded boat position prior to the current time
                step, expressed in meters (m) for linear, and radians (rad) for angular.
            vel (Vec4[Velocity, Frame]): The last recorded boat velocity prior to the current time
                step, expressed in meter per second (m/s) for linear, and radians per second
                (rad/s) for angular.
            timestep (float): The time interval on which the calculation is based, expressed in
                seconds (s).

        Returns:
            Vec4[Position, Frame]: The calculated next position of the boat, expressed in meters
                (m) for linear, and radians (rad) for angular.
        """
        return Vec4(pos.data + (vel.data * timestep))

    @staticmethod
    def next_velocity(
        vel: Vec4[Velocity, Frame],
        acc: Vec4[Acceleration, Frame],
        timestep: float,
    ) -> Vec4[Velocity, Frame]:
        """Calculates the boat's next velocity based on previous velocity, acceleration, and time
        step. Can be used for both linear and angular velocities.

        Args:
            vel (Vec4[Velocity, Frame]): The last recorded boat velocity prior to the current time
                step, expressed in meters per second (m/s) for linear, and radians per second
                (rad/s) for angular.
            acc (Vec4[Acceleration, Frame]): The last recorded boat acceleration prior to the
                current time step, expressed in meters per second squared (m/s^2) for linear, and
                radians per second squared (rad/s^2) for angular.
            timestep (float): The time interval on which the calculation is based, expressed in
                seconds (s).

        Returns:
            Vec4[Velocity, Frame]: The calculated next velocity of the boat, expressed in meters
                per second (m/s) for linear, and radians per second (rad/s) for angular.
        """
        return Vec4(vel.data + (acc.data * timestep))

    @staticmethod
    def next_lin_acceleration(
        mass: float, net_force: Vec4[Force, Frame]
    ) -> Vec4[Acceleration, Frame]:
        """Calculates the boat's next linear acceleration based on its mass and net force.

        Args:
            mass (float): The mass of the boat, expressed in kilograms (kg).
            net_force (Vec4[Force, Frame]): The net force acting on the boat, expressed in newtons
                (N).

        Returns:
            Vec4[Acceleration, Frame]: The calculated next linear acceleration of the boat,
                expressed in meters per second squared (m/s^2).
        """
        return Vec4(net_force.data / mass)

    @staticmethod
    def next_ang_acceleration(
        net_torque: Vec4[Torque, Frame], inertia_inverse: Mat4[InverseInertia, Frame]
    ) -> Vec4[Acceleration, Frame]:
        """Calculates the boat's next angular acceleration based on net torque and inverse of
        inertia.

        Args:
            net_torque (Vec4[Torque, Body]): The net torque acting on the boat, expressed in
                newton-meters (N•m).
            inertia_inverse (Mat4[InverseInertia, Body]): The inverse of the boat's inertia,
                expressed in per kilograms-meters squared (1/(kg•m^2)).

        Returns:
            Vec4[Acceleration, Body]: The calculated next angular acceleration of the boat,
                expressed in radians per second squared (rad/s^2).
        """
        return Vec4(inertia_inverse.data @ net_torque.data)
