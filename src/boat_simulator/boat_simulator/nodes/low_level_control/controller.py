"""Low level control logic for actuating the rudder and the sail."""

from abc import ABC, abstractmethod
from math import atan2, copysign, cos, sin
from typing import Tuple

import numpy as np

from boat_simulator.common.constants import RUDDER_MAX_ANGLE_RANGE, SAIL_MAX_ANGLE_RANGE
from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180


class ActuatorController(ABC):
    """Abstract class for rudder and sail actuation mechanisms.

    Attributes:
        `current_control_ang` (Scalar): Current control mechanism angle in degrees.
        `time_step` (Scalar): Time taken per iteration given in seconds.
        `control_speed` (Scalar): Speed of control angle change in degrees / second.
        'running_error' (Scalar): Error between current and target control angle.
    """

    def __init__(
        self,
        current_control_ang: Scalar,
        time_step: Scalar,
        control_speed: Scalar,
        running_error: Scalar,
        max_angle_range: Tuple[Scalar, Scalar],
    ):
        """Initializes the class attributes. Note that this class cannot be directly instantiated.

        Args:
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in seconds.
            `control_speed` (Scalar): Speed of control angle change in degrees / second.
            'running_error' (Scalar): Error between current and target control angle.
        """
        self.current_control_ang = current_control_ang
        self.time_step = time_step
        self.control_speed = control_speed
        self.running_error = running_error
        self.max_angle_range = max_angle_range

    def update_state(self) -> bool:
        """Updates the controller position iteratively based on control_speed * time_step, until
        the control target angle is reached.

        Args:
            `max_angle_range` (tuple): Max angle range individual to each different controller


        Returns:
            False if target control angle was not reached, else
            True if target control angle has been reached (running error = 0)
        """
        if np.isclose(self.running_error, 0, 0.01):
            return True
        else:
            change = copysign(1, self.running_error) * self.control_speed * self.time_step

        if abs(self.running_error) > change:
            next_control = max(self.current_control_ang + change, self.max_angle_range[0])
            self.running_error += change
        else:
            next_control = max(
                self.current_control_ang + self.running_error, self.max_angle_range[0]
            )
            self.running_error = 0
            return True

        self.current_control_ang = min(next_control, self.max_angle_range[1])
        return False

    @abstractmethod
    def reset_setpoint(self, new_target_angle: Scalar) -> None:
        """Resets a new target angle and computes a new corresponding setpoint

        Args:
            `new_target_angle` (Scalar): New target angle in degrees

        """
        pass


class RudderController(ActuatorController):
    """General Class for the Actuator Controller.

    RudderController Extends: ActuatorController
    """

    def __init__(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        current_control_ang: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
        control_speed: Scalar,
        running_error=0.0,
        max_angle_range=RUDDER_MAX_ANGLE_RANGE,
    ):
        """Initializes the class attributes.

        Args:
            `current_heading` (Scalar): Current boat heading direction in degrees,
             0 degrees (North) at the positive y axis and increasing clockwise.
            `desired_heading` (Scalar): Target boating heading direction in degrees,
             0 degrees (North) at the positive y axis and increasing clockwise.
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in seconds.
            `kp` (Scalar): Proportional constant when calculating error.
            `cp` (Scalar): Tuning parameter for control action.
            `control_speed` (Scalar): Speed of controller change in degrees per second.
            'running_error' (Scalar): Error between current and target control angle.

        """
        super().__init__(
            current_control_ang, time_step, control_speed, running_error, max_angle_range
        )

        self.current_heading = bound_to_180(current_heading)  # bound (-180, 180] in degrees
        self.desired_heading = bound_to_180(desired_heading)  # bound (-180, 180] in degrees
        self.kp = kp
        self.cp = cp
        self.setpoint = 0.0  # current setpoint angle in degrees

    def compute_error(self) -> Scalar:  # angle passed in as radians
        """Computes the error between desired and current heading
        implementation taken from: https://stackoverflow.com/a/2007279
        Angles are bound with the convention (-180, 180]

        Returns:
            Scalar: The error between the given headings in radians, 0 radians at positive
            y axis and increasing clockwise
        """
        desired_rad, current_rad = np.deg2rad(self.desired_heading), np.deg2rad(
            self.current_heading
        )

        error = atan2(sin(desired_rad - current_rad), cos(desired_rad - current_rad))

        return error  # in radians

    def compute_setpoint(self) -> Scalar:
        """Computes the corresponding control error angle between current control angle and
        target control angle. Uses Raye's implementation from:
            https://github.com/UBCSailbot/raye-boat-controller/blob/master/python/tack_controller.py

        Returns:
            Scalar: Corresponding error angle between the
            current and target control angle in degrees"""

        heading_error = self.compute_error()  # heading_error in radians

        if abs(heading_error) > np.sum(abs(np.deg2rad(RUDDER_MAX_ANGLE_RANGE))):
            raise ValueError("heading_error must be less than the total max_rudder angle range")

        rudder_setpoint = (self.kp * heading_error) / (1 + (self.cp * abs(heading_error)))

        if abs(rudder_setpoint) > RUDDER_MAX_ANGLE_RANGE[1]:
            raise ValueError("rudder_change must be between -pi/4 and pi/4")

        rudder_setpoint_deg = np.rad2deg(rudder_setpoint)  # in degrees
        self.running_error = rudder_setpoint_deg - self.current_control_ang  # in degrees
        self.setpoint = rudder_setpoint_deg  # in degrees

        return rudder_setpoint  # in degrees

    def reset_setpoint(self, new_desired_heading: Scalar) -> None:
        """Resets a new desired heading angle, therefore recalculating the corresponding
        setpoint and running error.

        Args:
            `new_desired_heading` (Scalar): New desired heading in degrees

        """

        self.desired_heading = new_desired_heading
        self.compute_setpoint()


class SailController(ActuatorController):
    """General Class for the Actuator Controller.

    SailController Extends: ActuatorController
    """

    def __init__(
        self,
        target_angle: Scalar,
        current_control_ang: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
        control_speed: Scalar,
        running_error=0.0,
        max_angle_range=SAIL_MAX_ANGLE_RANGE,
    ):
        """Initializes the class attributes.

        Args:
            `target_angle` (Scalar): Target angle for sail controller in degrees.
             0 degrees (North) at the positive y axis and increasing clockwise.
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in seconds.
            `kp` (Scalar): Proportional constant when calculating error.
            `cp` (Scalar): Tuning parameter for control action.
            `control_speed` (Scalar): Speed in which the controller turns in degrees / seconds.
            'running_error' (Scalar): Error between current and target control angle.

        """
        super().__init__(
            current_control_ang, time_step, control_speed, running_error, max_angle_range
        )

        self.target_angle = target_angle
        self.kp = kp
        self.cp = cp

    def compute_error(self):
        """Computes the corresponding control error angle between current control angle and
        target control angle

        Returns:
            Scalar: Corresponding error angle between the
            current and target control angle in degrees"""

        self.running_error = self.target_angle - self.current_control_ang

    def reset_setpoint(self, new_target: Scalar) -> None:
        """Resets a new desired sail actuator angle and updates the running_error

        Args:
            `target_angle` (Scalar): New desired sail controller angle

        """
        self.target_angle = new_target
        self.compute_error
