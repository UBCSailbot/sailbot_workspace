"""Low level control logic for actuating the rudder and the sail."""

from collections import deque
from math import atan2, copysign, cos, sin
from typing import Tuple

import numpy as np

from boat_simulator.common.angle_conventions import Heading, RudderAngle, TrimTabAngle
from boat_simulator.common.constants import RUDDER_MAX_ANGLE_RANGE, SAIL_MAX_ANGLE_RANGE
from boat_simulator.common.utils import bound_to_180


class ActuatorController:
    """Abstract class for rudder and sail actuation mechanisms.

    Attributes:
        `current_control_ang` (float): Current control mechanism angle in degrees.
        `time_step` (float): Time taken per iteration given in seconds.
        `control_speed` (float): Speed of control angle change in degrees / second.
        'max_angle_range' (Tuple): Max control angle range in degrees, minimum[0] and maximum[1]
    """

    def __init__(
        self,
        current_control_ang: float,
        time_step: float,
        control_speed: float,
        max_angle_range: Tuple[float, float],
    ):
        """Initializes the class attributes. Note that this class cannot be directly instantiated.

        Args:
            `current_control_ang` (float): Current control mechanism angle in degrees.
            `time_step` (float): Time per iteration given in seconds.
            `control_speed` (float): Speed of control angle change in degrees / second.
            'max_angle_range' (Tuple): Max control angle range in degrees,minimum[0] and maximum[1]
        """
        self.current_control_ang = current_control_ang
        self.time_step = time_step
        self.control_speed = control_speed
        self.max_angle_range = max_angle_range
        self.running_error = 0.0

    def update_state(self) -> bool:
        """Updates the controller position iteratively by control_speed * time_step, until
        the control target angle is reached.

        Returns:
            False if target control angle was not reached, else
            True if target control angle has been reached (running error = 0)
        """
        is_target_reached = False
        if np.isclose(self.running_error, 0, 0.01):
            return True

        change = copysign(self.control_speed * self.time_step, self.running_error)

        if abs(self.running_error) > abs(change):
            next_control = max(self.current_control_ang + change, self.max_angle_range[0])
            self.running_error -= change
        else:
            next_control = max(
                self.current_control_ang + self.running_error, self.max_angle_range[0]
            )
            self.running_error = 0
            is_target_reached = True

        self.current_control_ang = min(next_control, self.max_angle_range[1])
        return is_target_reached

    @property
    def is_target_reached(self) -> bool:
        return np.isclose(self.running_error, 0)


class RudderController(ActuatorController):
    """PID controller for the rudder.

    Attributes:
        `current_heading` (float): Current boat heading direction in degrees,
            0 degrees (North) at the positive y axis and increasing clockwise.
        `desired_heading` (float): Target boating heading direction in degrees,
            0 degrees (North) at the positive y axis and increasing clockwise.
        `current_control_ang` (float): Last commanded rudder angle in degrees.
        `time_step` (float): Time per iteration given in seconds.
        `kp`, `ki`, `kd` (float): PID constants for the rudder controller.
        `cp` (float): Tuning parameter for control action.
        `buffer_size` (int): Number of samples used to filter the derivative error.
        `max_angle_range` (Tuple): Max rudder control angle range in degrees, minimum[0] and
            maximum[1]

    """

    def __init__(
        self,
        current_heading: float,
        desired_heading: float,
        current_control_ang: float,
        time_step: float,
        kp: float,
        ki: float,
        kd: float,
        cp: float,
        buffer_size: int = 1,
        max_angle_range=RUDDER_MAX_ANGLE_RANGE,
    ):
        """Initializes the class attributes.

        Args:
            `current_heading` (float): Current boat heading direction in degrees,
             0 degrees (North) at the positive y axis and increasing clockwise.
            `desired_heading` (float): Target boating heading direction in degrees,
             0 degrees (North) at the positive y axis and increasing clockwise.
            `current_control_ang` (float): Current control mechanism angle in degrees.
            `time_step` (float): Time per control-loop iteration in seconds.
            `kp` (float): Proportional gain.
            `ki` (float): Integral gain.
            `kd` (float): Derivative gain.
            `cp` (float): Tuning parameter for the saturating proportional term.
            `buffer_size` (int): Number of past error samples used to filter the
             derivative term. Defaults to 1 (unfiltered derivative).
            `max_angle_range` (Tuple): Max control angle range in degrees,
             minimum[0] and maximum[1]
        """
        self.current_control_ang = current_control_ang
        self.time_step = time_step
        self.max_angle_range = max_angle_range

        self.current_heading = bound_to_180(current_heading)
        self.desired_heading = bound_to_180(desired_heading)
        self.setpoint = 0.0  # last commanded rudder angle in degrees

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.cp = cp
        self.buffer_size = buffer_size

        self._integral_error = 0.0
        self._error_buffer: deque = deque(maxlen=max(self.buffer_size, 1))

        self.reset_setpoint(self.desired_heading, self.current_heading)

    def reset_setpoint(self, new_desired_heading: float, new_current_heading: float) -> None:
        """Resets a new desired heading angle, therefore recalculating the corresponding
        setpoint and running error.

        Args:
            `new_desired_heading` (float): New desired heading in degrees
            `new_current_heading` (float): New current heading in degrees
        """

        self.desired_heading = bound_to_180(new_desired_heading)
        self.current_heading = bound_to_180(new_current_heading)
        self._integral_error = 0.0
        self._error_buffer.clear()
        self._error_buffer.append(self._compute_error())
        self._compute_setpoint()

    def step(self, new_current_heading: float) -> float:
        """Updates the controller state with a new current heading and computes the next rudder
        setpoint.

        Args:
            `new_current_heading` (float): New current heading in degrees

        Returns:
            float: The next rudder setpoint in degrees
        """
        self.current_heading = bound_to_180(new_current_heading)
        self.current_control_ang = self._compute_setpoint()
        return self.current_control_ang

    def _compute_error(self) -> float:
        """Computes the error between desired and current heading
        implementation taken from: https://stackoverflow.com/a/2007279
        Angles are bound with the convention (-180, 180]

        Returns:
            float: The error between the given headings in radians, 0 radians at positive
            y axis and increasing clockwise
        """
        desired_rad, current_rad = np.deg2rad(self.desired_heading), np.deg2rad(
            self.current_heading
        )

        error_rad = atan2(sin(desired_rad - current_rad), cos(desired_rad - current_rad))

        return error_rad

    def _compute_setpoint(self) -> float:
        """Computes the next rudder setpoint based on the current and desired heading.

        Returns:
            float: The next rudder setpoint in degrees
        """
        error_rad = self._compute_error()
        self._error_buffer.append(error_rad)

        p_term = (self.kp * error_rad) / (1 + self.cp * abs(error_rad))

        self._integral_error += error_rad * self.time_step
        i_term = self.ki * self._integral_error

        if len(self._error_buffer) >= 2:
            d_term = (
                self.kd
                * (self._error_buffer[-1] - self._error_buffer[0])
                / (self.time_step * (len(self._error_buffer) - 1))
            )
        else:
            d_term = 0.0

        calculated_rad = p_term + i_term + d_term

        min_rad = np.deg2rad(self.max_angle_range[0])
        max_rad = np.deg2rad(self.max_angle_range[1])
        output_rad = min(max(calculated_rad, min_rad), max_rad)

        if output_rad != calculated_rad:
            self._integral_error -= error_rad * self.time_step

        self.setpoint = np.rad2deg(output_rad)
        return self.setpoint

    def _change_desired_heading(self, changed_desired_heading) -> None:
        """Changes desired heading to a new angle. Used for testing purposes

        Args:
            `changed_desired_heading` (float): New desired heading in degrees

        """
        self.desired_heading = changed_desired_heading

    @property
    def current_heading_angle(self) -> Heading:
        """Current NED compass heading as a unit-checked :class:`Heading`."""
        return Heading.from_degrees(self.current_heading)

    @property
    def desired_heading_angle(self) -> Heading:
        """Desired NED compass heading as a unit-checked :class:`Heading`."""
        return Heading.from_degrees(self.desired_heading)

    @property
    def rudder_setpoint(self) -> RudderAngle:
        """Target rudder angle as a range-checked :class:`RudderAngle` (±30°)."""
        return RudderAngle.from_degrees(self.setpoint)

    @property
    def current_rudder_angle(self) -> RudderAngle:
        """Current rudder angle as a range-checked :class:`RudderAngle` (±30°)."""
        return RudderAngle.from_degrees(self.current_control_ang)


class SailController(ActuatorController):
    """General Class for the Actuator Controller.

    SailController Extends: ActuatorController
    """

    def __init__(
        self,
        target_angle: float,
        current_control_ang: float,
        time_step: float,
        control_speed: float,
        max_angle_range=SAIL_MAX_ANGLE_RANGE,
    ):
        """Initializes the class attributes.

        Args:
            `target_angle` (float): Target angle for the trim tab in degrees.
             0 degrees (North) at the positive y axis and increasing clockwise.
            `current_control_ang` (float): Current control mechanism angle in degrees.
            `time_step` (float): Time per iteration given in seconds.
            `control_speed` (float): Speed in which the controller turns in degrees / seconds.
            'max_angle_range' (Tuple): Trim tab angle range in degrees,minimum[0] and maximum[1].

        """
        super().__init__(
            current_control_ang,
            time_step,
            control_speed,
            max_angle_range,
        )
        self.target_angle = target_angle
        self.reset_setpoint(self.target_angle)

    def reset_setpoint(self, new_target: float) -> None:
        """Resets a new desired trim tab angle and updates the running_error

        Args:
            `target_angle` (float): New desired sail controller angle in degrees

        """
        self.target_angle = new_target
        self._compute_error()

    def _compute_error(self) -> float:
        """Computes the corresponding control error angle between current control angle and
        target control angle

        Returns:
            float: Corresponding error angle between the
            current and target control angle in degrees"""

        self.running_error = self.target_angle - self.current_control_ang
        return self.running_error

    @property
    def trim_tab_setpoint(self) -> TrimTabAngle:
        """Target trim-tab angle as a range-checked :class:`TrimTabAngle` (±40°)."""
        return TrimTabAngle.from_degrees(self.target_angle)

    @property
    def current_trim_tab_angle(self) -> TrimTabAngle:
        """Current trim-tab angle as a range-checked :class:`TrimTabAngle` (±40°)."""
        return TrimTabAngle.from_degrees(self.current_control_ang)
