"""Formulas for estimating the next position, velocity, and acceleration."""

from numpy.typing import NDArray

from boat_simulator.common.types import Scalar


class KinematicsFormulas:
    """Contains formulas for calculating the boat's next position, velocity, and acceleration based
    on previous kinematic data."""

    @staticmethod
    def next_position(pos: NDArray, vel: NDArray, acc: NDArray, timestep: Scalar) -> NDArray:
        """Calculates the boat's next position based on previous data and time step. Can be used
        for both linear and angular positions.

        Args:
            pos (NDArray): The last recorded boat position prior to the current time step,
                expressed in meters (m) for linear, and radians (rad) for angular.
            vel (NDArray): The last recorded boat velocity prior to the current time step,
                expressed in meter per second (m/s) for linear, and radians per second (rad/s) for
                angular.
            acc (NDArray): The last recorded boat acceleration prior to the current time step,
                expressed in meters per second squared (m/s^2) for linear, and radians per second
                squared (rad/s^2) for angular.
            timestep (Scalar): The time interval on which the calculation is based, expressed in
                seconds (s).

        Returns:
            NDArray: The calculated next position of the boat, expressed in meters (m) for linear,
                and radians (rad) for angular.
        """
        return pos + (vel * timestep) + (acc * (timestep**2 / 2))

    @staticmethod
    def next_velocity(vel: NDArray, acc: NDArray, timestep: Scalar) -> NDArray:
        """Calculates the boat's next velocity based on previous velocity, acceleration, and time
        step. Can be used for both linear and angular velocities.

        Args:
            vel (NDArray): The last recorded boat velocity prior to the current time step,
                expressed in meters per second (m/s) for linear, and radians per second (rad/s) for
                angular.
            acc (NDArray): The last recorded boat acceleration prior to the current time step,
                expressed in meters per second squared (m/s^2) for linear, and radians per second
                squared (rad/s^2) for angular.
            timestep (Scalar): The time interval on which the calculation is based, expressed in
                seconds (s).

        Returns:
            NDArray: The calculated next velocity of the boat, expressed in meters per second (m/s)
                for linear, and radians per second (rad/s) for angular.
        """
        return vel + (acc * timestep)

    @staticmethod
    def next_lin_acceleration(mass: Scalar, net_force: NDArray) -> NDArray:
        """Calculates the boat's next linear acceleration based on its mass and net force.

        Args:
            mass (float): The mass of the boat, expressed in kilograms (kg).
            net_force (NDArray): The net force acting on the boat, expressed in newtons (N).

        Returns:
            NDArray: The calculated next linear acceleration of the boat, expressed in meters per
                second squared (m/s^2).
        """
        return net_force / mass

    @staticmethod
    def next_ang_acceleration(net_torque: NDArray, inertia_inverse: NDArray) -> NDArray:
        """Calculates the boat's next angular acceleration based on net torque and inverse of
        inertia.

        Args:
            net_torque (NDArray): The net torque acting on the boat, expressed in newton-meters
                (N•m).
            inertia_inverse (NDArray): The inverse of the boat's inertia, expressed in
                kilograms-meters squared (kg•m^2).

        Returns:
            NDArray: The calculated next angular acceleration of the boat, expressed in radians per
                second squared (rad/s^2).
        """
        return inertia_inverse @ net_torque
