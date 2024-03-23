"""This module provides functionality for computing the lift and drag forces acting on a medium."""

from typing import Tuple

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.utils import Scalar


class MediumForceComputation:
    """This class calculates the lift and drag forces experienced by a medium when subjected to
    fluid flow.

    Attributes:
        `lift_coefficients` (NDArray): An array of shape (n, 2) where each row contains a pair
            (x, y) representing an angle of attack, in degrees, and its corresponding lift
            coefficient.
        `drag_coefficients` (NDArray): An array of shape (n, 2) where each row contains a pair
            (x, y) representing an angle of attack, in degrees, and its corresponding drag
            coefficient.
        `areas` (Scalar): Corresponding area, in square meters (m^2).
        `fluid_density` (Scalar): The density of the fluid acting on the medium, expressed in
            kilograms per cubic meter (kg/m^3).
    """

    def __init__(
        self,
        lift_coefficients: NDArray,
        drag_coefficients: NDArray,
        areas: Scalar,
        fluid_density: Scalar,
    ):
        self.__lift_coefficients = lift_coefficients
        self.__drag_coefficients = drag_coefficients
        self.__areas = areas
        self.__fluid_density = fluid_density

    def calculate_attack_angle(self, apparent_velocity: NDArray, orientation: Scalar) -> Scalar:
        """Calculates the angle of attack formed between the orientation angle of the medium
        and the direction of the apparent velocity, bounded between -180 and 180 degrees.

        Args:
            apparent_velocity (NDArray): The apparent (relative) velocity between the fluid and
                the medium, expressed in meters per second (m/s).
            orientation (Scalar): The orientation angle of the medium in degrees.

        Returns:
            Scalar: The angle of attack formed between the orientation angle of the medium and
                the direction of the apparent velocity, expressed in degrees
                and bounded between -180 and 180 degrees.
        """
        # Check if the apparent velocity is [0, 0]
        if np.all(apparent_velocity == 0):
            # Directly return the normalized orientation as the angle of attack
            # Normalize orientation to be within [-180, 180)
            return ((orientation + 180) % 360) - 180

        # Calculate the angle in degrees of the apparent velocity
        angle_of_attack_raw = np.rad2deg(np.arctan2(apparent_velocity[1], apparent_velocity[0]))

        # Adjust orientation to be in the range of [-180, 180)
        orientation = ((orientation + 180) % 360) - 180

        # Calculate the raw angle of attack by subtracting the orientation from the velocity angle
        angle_of_attack = angle_of_attack_raw - orientation

        # Normalize the angle of attack to [-180, 180) range
        angle_of_attack = ((angle_of_attack + 180) % 360) - 180

        return angle_of_attack

    def compute(self, apparent_velocity: NDArray, orientation: Scalar) -> Tuple[NDArray, NDArray]:
        """Computes the lift and drag forces experienced by a medium immersed in a fluid.

        Args:
            apparent_velocity (NDArray): The apparent (relative) velocity between the fluid and the
                medium, calculated as the difference between the fluid velocity and the medium
                velocity (fluid_velocity - medium_velocity), expressed in meters per second (m/s).
            orientation (Scalar): The orientation angle of the medium in degrees, where 0 degrees
                corresponds to the positive x-axis, and angles increase counter-clockwise (CCW).

        Returns:
            Tuple[NDArray, NDArray]: A tuple containing the lift force and drag force experienced
                by the medium, both expressed in newtons (N).
        """

        attack_angle = self.calculate_attack_angle(apparent_velocity, orientation)
        lift_coefficient, drag_coefficient = self.interpolate(attack_angle)
        velocity_magnitude = np.linalg.norm(apparent_velocity)

        # Calculate the lift and drag forces

        lift_force_magnitude = self.__calculate_fluid_force_magnitude(
            lift_coefficient, velocity_magnitude
        )
        drag_force_magnitude = self.__calculate_fluid_force_magnitude(
            drag_coefficient, velocity_magnitude
        )

        drag_force_unit_vector = apparent_velocity / velocity_magnitude
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
            # Should not happen if drag force direction is properly normalized
            # This could be a fallback for an unexpected case
            lift_force_direction = np.array([0, 0])

        # Rotate the lift and drag forces back to the original orientation
        lift_force_direction = self.__rotate_vector(
            lift_force_direction, orientation, clockwise=False
        )
        drag_force_unit_vector = self.__rotate_vector(
            drag_force_unit_vector, orientation, clockwise=False
        )

        lift_force = lift_force_magnitude * lift_force_direction
        drag_force = drag_force_magnitude * drag_force_unit_vector

        return lift_force, drag_force

    def __calculate_fluid_force_magnitude(
        self, coefficient: Scalar, velocity_magnitude: Scalar
    ) -> Scalar:
        """Calculates the magnitude of fluid forces based on coefficient and velocity."""
        return 0.5 * self.__fluid_density * coefficient * self.__areas * (velocity_magnitude**2)

    def __rotate_vector(self, v: NDArray, theta_degrees: Scalar, clockwise=True) -> NDArray:
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

    def interpolate(self, attack_angle: Scalar) -> Tuple[Scalar, Scalar, Scalar]:
        """Performs linear interpolation to estimate the lift and drag coefficients, as well as the
        associated area upon which the fluid acts, based on the provided angle of attack.

            Args:
                attack_angle (Scalar): The angle of attack formed between the orientation angle of
                    the medium and the direction of the apparent velocity, expressed in degrees.

            Returns:
                Tuple[Scalar, Scalar, Scalar]: A tuple representing the computed parameters. The
                    first scalar denotes the lift coefficient, the second scalar represents the
                    drag coefficient, and the third scalar indicates the surface area upon which
                    the fluid acts. Both lift and drag coefficients are unitless, while the
                    area is expressed in square meters (m^2).
        """

        lift_coefficient = np.interp(
            attack_angle, self.__lift_coefficients[:, 0], self.__lift_coefficients[:, 1]
        )
        drag_coefficient = np.interp(
            attack_angle, self.__drag_coefficients[:, 0], self.__drag_coefficients[:, 1]
        )
        return lift_coefficient, drag_coefficient

    def _draw_boat(ax, position, orientation):
        """Draws a simplified boat shape on the given axes, ensuring it aligns
        with the orientation line."""
        boat_length = 1.0
        boat_width = 0.3

        # Center the shape around (0, 0)
        front_extension = 3 * boat_length / 4
        rear_extension = -boat_length / 4

        # Calculate the offset to center the shape
        offset = front_extension + rear_extension

        boat_shape = np.array(
            [
                [front_extension - offset, 0],
                [boat_length / 2 - offset, boat_width / 2],
                [rear_extension - offset, 0],
                [boat_length / 2 - offset, -boat_width / 2],
                [front_extension - offset, 0],
            ]
        )

        # Rotation matrix for anticlockwise rotation
        rotation_matrix = np.array(
            [
                [np.cos(np.deg2rad(orientation)), np.sin(np.deg2rad(orientation))],
                [np.sin(np.deg2rad(orientation)), np.cos(np.deg2rad(orientation))],
            ]
        )

        # Apply rotation
        rotated_boat = np.dot(boat_shape, rotation_matrix)

        # Translate boat to its position
        translated_boat = rotated_boat + np.array(position)

        # Draw the boat
        ax.plot(translated_boat[:, 0], translated_boat[:, 1], "k")

        # Ensure the orientation line is drawn correctly
        # Calculate a point along the orientation direction
        direction = np.array([np.cos(np.deg2rad(orientation)), np.sin(np.deg2rad(orientation))])
        line_start = np.array(position)
        line_end = (
            line_start + direction * boat_length
        )  # Extend the line out from the boat's position

        # Draw orientation line
        ax.plot(
            [line_start[0], line_end[0]], [line_start[1], line_end[1]], "black", linestyle="--"
        )

    def visualize_forces(
        self, apparent_velocity, lift_force, drag_force, position=[0, 0], orientation=0
    ):
        """Visualizes the sailboat, apparent velocity, lift force, and drag force."""
        fig, ax = plt.subplots()
        attack_angle = self.calculate_attack_angle(apparent_velocity, orientation)
        # Normalize forces for visualization
        norm_apparent_velocity = apparent_velocity / np.linalg.norm(apparent_velocity)
        norm_lift_force = lift_force / np.linalg.norm(lift_force)
        norm_drag_force = drag_force / np.linalg.norm(drag_force)

        # Draw the boat
        MediumForceComputation._draw_boat(ax, position, orientation)
        # Plot forces and velocity
        ax.quiver(
            position[0],
            position[1],
            norm_apparent_velocity[0],
            norm_apparent_velocity[1],
            color="blue",
            scale=5,
            label="Apparent Velocity",
            pivot="tip",
        )
        ax.quiver(
            position[0],
            position[1],
            norm_lift_force[0],
            norm_lift_force[1],
            color="red",
            scale=5,
            label="Lift Force",
        )
        ax.quiver(
            position[0],
            position[1],
            norm_drag_force[0],
            norm_drag_force[1],
            color="green",
            scale=5,
            label="Drag Force",
        )
        orientation_rad = np.deg2rad(orientation)  # Convert orientation to radians
        ax.axline((0, 0), slope=np.tan(orientation_rad), color="black", linestyle="--")

        # Calculate angle for drag force
        drag_angle = np.arctan2(norm_drag_force[1], norm_drag_force[0])

        # Determine start and end angles for the arc
        start_angle = np.rad2deg(orientation_rad)
        end_angle = np.rad2deg(drag_angle)

        # Draw arc to represent angle between orientation and drag force
        radius = 0.05
        arc = patches.Arc(
            position,
            2 * radius,
            2 * radius,
            angle=0,
            theta1=min(start_angle, end_angle),
            theta2=max(start_angle, end_angle),
            color="purple",
            label="Angle Arc",
        )
        ax.add_patch(arc)

        ax.axis("equal")
        ax.legend()
        plt.title("Forces Acting on Sailboat for Attack Angle: " + str(round(attack_angle)))
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.grid(True)
        plt.show()

    @property
    def lift_coefficients(self) -> NDArray:
        return self.__lift_coefficients

    @property
    def drag_coefficients(self) -> NDArray:
        return self.__drag_coefficients

    @property
    def areas(self) -> Scalar:
        return self.__areas

    @property
    def fluid_density(self) -> Scalar:
        return self.__fluid_density
