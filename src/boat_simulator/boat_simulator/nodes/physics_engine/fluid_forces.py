"""This module provides functionality for computing the lift and drag forces acting on a medium."""

from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from rclpy.logging import get_logger

from boat_simulator.common.conventions import Body, Force, Velocity
from boat_simulator.common.types import CoeffTable, Vec2

_logger = get_logger(__name__)


class MediumForceComputation:
    """This class calculates the lift and drag forces experienced by a medium when subjected to
    fluid flow.

    Attributes:
        `lift_coefficients` (CoeffTable): An array of shape (n, 2) where each row contains a pair
            (x, y) representing an angle of attack, in degrees, and its corresponding lift
            coefficient.
        `drag_coefficients` (CoeffTable): An array of shape (n, 2) where each row contains a pair
            (x, y) representing an angle of attack, in degrees, and its corresponding drag
            coefficient.
        `areas` (float): A 2D area of the aero/hydrofoil.
        `fluid_density` (float): The density of the fluid acting on the medium, expressed in
            kilograms per cubic meter (kg/m^3).
    """

    def __init__(
        self,
        lift_coefficients: CoeffTable,
        drag_coefficients: CoeffTable,
        areas: float,
        fluid_density: float,
    ):
        self.__lift_coefficients = lift_coefficients
        self.__drag_coefficients = drag_coefficients
        self.__areas = areas
        self.__fluid_density = fluid_density

    def calculate_attack_angle(
        self, apparent_velocity: Vec2[Velocity, Body], orientation: float
    ) -> float:
        """Calculates the angle of attack formed between the orientation angle of the medium
        and the direction of the apparent velocity, bounded between -180 and 180 degrees.

        Args:
            apparent_velocity (Vec2[Velocity, Body]): The apparent (relative) velocity between the
                fluid and the medium, expressed in meters per second (m/s).
            orientation (float): The orientation angle of the medium in degrees.

        Returns:
            float: The angle of attack formed between the orientation angle of the medium and
                the direction of the apparent velocity, expressed in degrees
                and bounded between -180 and 180 degrees.
        """
        velocity = apparent_velocity.data
        # Check if the apparent velocity is [0, 0]
        if np.all(velocity == 0):
            # Directly return the normalized orientation as the angle of attack
            # Normalize orientation to be within [-180, 180)
            angle_of_attack = ((orientation + 180) % 360) - 180
            _logger.debug(
                f"calculate_attack_angle: zero velocity, returning orientation={orientation:.2f} "
                + f"as aoa={angle_of_attack:.2f}",
            )
            return angle_of_attack

        # Calculate the angle in degrees of the apparent velocity
        angle_of_attack_raw = np.rad2deg(np.arctan2(velocity[1], velocity[0]))

        # Adjust orientation to be in the range of [-180, 180)
        orientation = ((orientation + 180) % 360) - 180

        # Calculate the raw angle of attack by subtracting the orientation from the velocity angle
        angle_of_attack = angle_of_attack_raw - orientation

        # Normalize the angle of attack to [-180, 180) range
        angle_of_attack = ((angle_of_attack + 180) % 360) - 180

        _logger.debug(
            f"calculate_attack_angle: vel={velocity} orientation={orientation:.2f} "
            + f"raw={angle_of_attack_raw:.2f} aoa={angle_of_attack:.2f}"
        )
        return angle_of_attack

    def compute(
        self, apparent_velocity: Vec2[Velocity, Body], orientation: float
    ) -> Tuple[Vec2[Force, Body], Vec2[Force, Body]]:
        """Computes the lift and drag forces experienced by a medium immersed in a fluid.

        Args:
            apparent_velocity (Vec2[Velocity, Body]): The apparent (relative) velocity between the
                fluid and the medium, calculated as the difference between the fluid velocity and
                the medium velocity (fluid_velocity - medium_velocity), expressed in meters per
                second (m/s).
            orientation (float): The orientation angle of the medium in degrees, where 0 degrees
                corresponds to the positive x-axis, and angles increase counter-clockwise (CCW).

        Returns:
            Tuple[Vec2[Force, Body], Vec2[Force, Body]]: A tuple containing the lift force and drag
                force experienced by the medium, both expressed in newtons (N).
        """

        attack_angle = self.calculate_attack_angle(apparent_velocity, orientation)
        lift_coefficient, drag_coefficient, area = self.interpolate(attack_angle)
        velocity = apparent_velocity.data
        velocity_magnitude = np.linalg.norm(velocity)

        # With no relative flow there is no lift or drag (force ∝ |v|²).
        # Returning early also avoids the division by velocity_magnitude below, which
        # would otherwise produce NaN/Inf forces that propagate irreversibly through the
        # kinematics and blow up the simulation.
        if velocity_magnitude == 0:
            zero_force: Vec2[Force, Body] = Vec2.from_xy(0.0, 0.0)
            _logger.info("compute: zero apparent velocity, returning zero lift/drag force")
            return zero_force, zero_force

        _logger.info(
            f"compute: apparent_vel={velocity} m/s orientation={orientation:6.2f}° "
            f"aoa={attack_angle:6.2f}° "
            f"C_l={lift_coefficient:6.3f} C_d={drag_coefficient:6.3f} "
            f"area={area:.3f} m² |v|={velocity_magnitude:6.2f} m/s "
            f"L={0.5 * self.__fluid_density * lift_coefficient * area * velocity_magnitude**2} N "
            f"D={0.5 * self.__fluid_density * drag_coefficient * area * velocity_magnitude**2} N"
        )

        # Calculate the lift and drag forces

        lift_force_magnitude = self.__calculate_fluid_force_magnitude(
            lift_coefficient, velocity_magnitude, area
        )
        drag_force_magnitude = self.__calculate_fluid_force_magnitude(
            drag_coefficient, velocity_magnitude, area
        )

        drag_force_unit_vector = velocity / velocity_magnitude
        drag_force_unit_vector = self.__rotate_vector(drag_force_unit_vector, orientation)

        # Rotate the lift and drag forces by 90 degrees to obtain the lift and drag forces

        # Convention used here is that the positive x-axis is 0 degrees
        # and the positive y-axis is 90 degrees
        # Positive rotation is counter clockwise

        is_drag_in_first_or_third_quadrant = (
            drag_force_unit_vector[0] > 0 and drag_force_unit_vector[1] > 0
        ) or (drag_force_unit_vector[0] < 0 and drag_force_unit_vector[1] < 0)

        is_drag_in_second_or_fourth_quadrant = (
            drag_force_unit_vector[0] > 0 and drag_force_unit_vector[1] < 0
        ) or (drag_force_unit_vector[0] < 0 and drag_force_unit_vector[1] > 0)

        # Rotate the lift force direction based on the quadrant of the drag force
        if is_drag_in_first_or_third_quadrant:
            # Rotate counter clockwise to get lift direction
            lift_force_direction = np.array(
                [-drag_force_unit_vector[1], drag_force_unit_vector[0]]
            )
        elif is_drag_in_second_or_fourth_quadrant:
            # Rotate clockwise to get lift direction
            lift_force_direction = np.array(
                [drag_force_unit_vector[1], -drag_force_unit_vector[0]]
            )
        else:
            # The drag unit vector lies exactly on an axis (one component is 0), so it belongs to
            # neither quadrant test above. Use the CCW perpendicular as a consistent default rather
            # than zeroing the lift direction, which would silently discard all lift force.
            lift_force_direction = np.array(
                [-drag_force_unit_vector[1], drag_force_unit_vector[0]]
            )

        # Rotate the lift and drag forces back to the original orientation
        lift_force_direction = self.__rotate_vector(
            lift_force_direction, orientation, clockwise=False
        )
        drag_force_unit_vector = self.__rotate_vector(
            drag_force_unit_vector, orientation, clockwise=False
        )

        lift_force = np.asarray(lift_force_magnitude * lift_force_direction, dtype=np.float64)
        drag_force = np.asarray(drag_force_magnitude * drag_force_unit_vector, dtype=np.float64)

        _logger.info(
            f"compute: lift_force={lift_force} "
            f"lift_force_direction={lift_force_direction} drag_force={drag_force} "
            f"drag_force_unit_vector={drag_force_unit_vector}"
        )

        return Vec2(lift_force), Vec2(drag_force)

    def __calculate_fluid_force_magnitude(
        self, coefficient: float, velocity_magnitude: float, area: float
    ) -> float:
        """Calculates the magnitude of fluid forces based on coefficient, velocity, and area."""
        return 0.5 * self.__fluid_density * coefficient * area * (velocity_magnitude**2)

    def __rotate_vector(self, v: NDArray, theta_degrees: float, clockwise=True) -> NDArray:
        """
        Rotates a vector by a specified angle in degrees.

        Args:
        v (np.array): The vector to be rotated.
        theta_degrees (float): The rotation angle in degrees.
        clockwise (bool, optional): Determines the direction of rotation. If True (default),
                                    rotates the vector clockwise. If False, rotates the vector
                                    counterclockwise.

        Returns:
            np.array: The rotated vector.
        """
        theta_radians = np.deg2rad(theta_degrees)
        sign = 1 if clockwise else -1
        rotation_matrix = np.array(
            [
                [np.cos(theta_radians), sign * np.sin(theta_radians)],
                [-sign * np.sin(theta_radians), np.cos(theta_radians)],
            ]
        )
        v_rotated = np.dot(rotation_matrix, v)
        return v_rotated

    def interpolate(self, attack_angle: float) -> Tuple[float, float, float]:
        """Performs linear interpolation to estimate the lift coefficient, drag coefficient, and
        area upon which the fluid acts, based on the provided angle of attack.

            Args:
                attack_angle (float): The angle of attack formed between the orientation angle of
                    the medium and the direction of the apparent velocity, expressed in degrees.

            Returns:
                Tuple[float, float, float]: A tuple of (lift_coefficient, drag_coefficient,
                    area). Both coefficients are unitless; area is in square meters (m^2).
        """

        # The foils are symmetric, so the lookup tables only store the positive AoA branch:
        # C_l is odd (C_l(-a) = -C_l(a)) and C_d is even (C_d(-a) = C_d(a)). We interpolate on
        # |AoA| and re-apply the sign to lift. Beyond the table's max angle the foil is fully
        # stalled and outside the modeled regime, so we return zero (np.interp would otherwise
        # silently clamp to the endpoint, producing peak lift at all out-of-range angles →
        # runaway thrust). Tables are assumed to start at 0°.
        abs_attack_angle = abs(attack_angle)
        lift_sign = float(np.sign(attack_angle))

        if abs_attack_angle > self.__lift_coefficients.max_angle:
            lift_coefficient = 0.0
        else:
            lift_coefficient = lift_sign * self.__lift_coefficients.interpolate(abs_attack_angle)

        if abs_attack_angle > self.__drag_coefficients.max_angle:
            drag_coefficient = 0.0
        else:
            drag_coefficient = self.__drag_coefficients.interpolate(abs_attack_angle)

        # The foil area is modeled as constant (independent of angle of attack).
        area = self.__areas

        return lift_coefficient, drag_coefficient, area

    @property
    def lift_coefficients(self) -> CoeffTable:
        return self.__lift_coefficients

    @property
    def drag_coefficients(self) -> CoeffTable:
        return self.__drag_coefficients

    @property
    def areas(self) -> float:
        return self.__areas

    @property
    def fluid_density(self) -> float:
        return self.__fluid_density
