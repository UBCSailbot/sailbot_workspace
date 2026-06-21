"""Formulas for estimating the next position, velocity, and acceleration."""

from numpy.typing import NDArray

from boat_simulator.common.frames import (
    Acceleration,
    Body,
    Force,
    Frame,
    Position,
    Torque,
    Vec3,
    Velocity,
)
from boat_simulator.common.types import Scalar


class KinematicsFormulas:
    """Contains formulas for calculating the boat's next position, velocity, and acceleration based
    on previous kinematic data.

    Every formula operates on and returns :class:`Vec3` values so the reference frame and
    physical quantity carried by each vector is preserved through the integration. Arithmetic is
    performed on the underlying ``.data`` arrays because the integration deliberately combines
    vectors of different quantities (e.g. position += velocity·dt), which the quantity-tagged
    ``Vec3`` operators intentionally forbid.
    """

    @staticmethod
    def next_position(
        pos: Vec3[Position, Frame],
        vel: Vec3[Velocity, Frame],
        acc: Vec3[Acceleration, Frame],
        timestep: Scalar,
    ) -> Vec3[Position, Frame]:
        """Calculates the boat's next position based on previous data and time step. Can be used
        for both linear and angular positions.

        Args:
            pos (Vec3[Position, Frame]): The last recorded boat position prior to the current time
                step, expressed in meters (m) for linear, and radians (rad) for angular.
            vel (Vec3[Velocity, Frame]): The last recorded boat velocity prior to the current time
                step, expressed in meter per second (m/s) for linear, and radians per second
                (rad/s) for angular.
            acc (Vec3[Acceleration, Frame]): The last recorded boat acceleration prior to the
                current time step, expressed in meters per second squared (m/s^2) for linear, and
                radians per second squared (rad/s^2) for angular.
            timestep (Scalar): The time interval on which the calculation is based, expressed in
                seconds (s).

        Returns:
            Vec3[Position, Frame]: The calculated next position of the boat, expressed in meters
                (m) for linear, and radians (rad) for angular.
        """
        return Vec3(pos.data + (vel.data * timestep) + (acc.data * (timestep**2 / 2)))

    @staticmethod
    def next_velocity(
        vel: Vec3[Velocity, Frame],
        acc: Vec3[Acceleration, Frame],
        timestep: Scalar,
    ) -> Vec3[Velocity, Frame]:
        """Calculates the boat's next velocity based on previous velocity, acceleration, and time
        step. Can be used for both linear and angular velocities.

        Args:
            vel (Vec3[Velocity, Frame]): The last recorded boat velocity prior to the current time
                step, expressed in meters per second (m/s) for linear, and radians per second
                (rad/s) for angular.
            acc (Vec3[Acceleration, Frame]): The last recorded boat acceleration prior to the
                current time step, expressed in meters per second squared (m/s^2) for linear, and
                radians per second squared (rad/s^2) for angular.
            timestep (Scalar): The time interval on which the calculation is based, expressed in
                seconds (s).

        Returns:
            Vec3[Velocity, Frame]: The calculated next velocity of the boat, expressed in meters
                per second (m/s) for linear, and radians per second (rad/s) for angular.
        """
        return Vec3(vel.data + (acc.data * timestep))

    @staticmethod
    def next_lin_acceleration(
        mass: Scalar, net_force: Vec3[Force, Frame]
    ) -> Vec3[Acceleration, Frame]:
        """Calculates the boat's next linear acceleration based on its mass and net force.

        Args:
            mass (Scalar): The mass of the boat, expressed in kilograms (kg).
            net_force (Vec3[Force, Frame]): The net force acting on the boat, expressed in newtons
                (N).

        Returns:
            Vec3[Acceleration, Frame]: The calculated next linear acceleration of the boat,
                expressed in meters per second squared (m/s^2).
        """
        return Vec3(net_force.data / mass)

    @staticmethod
    def next_ang_acceleration(
        net_torque: Vec3[Torque, Body], inertia_inverse: NDArray
    ) -> Vec3[Acceleration, Body]:
        """Calculates the boat's next angular acceleration based on net torque and inverse of
        inertia.

        Args:
            net_torque (Vec3[Torque, Body]): The net torque acting on the boat, expressed in
                newton-meters (N•m).
            inertia_inverse (NDArray): The inverse of the boat's inertia, expressed in per
                kilograms-meters squared (1/(kg•m^2)).

        Returns:
            Vec3[Acceleration, Body]: The calculated next angular acceleration of the boat,
                expressed in radians per second squared (rad/s^2).
        """
        return Vec3(inertia_inverse @ net_torque.data)
