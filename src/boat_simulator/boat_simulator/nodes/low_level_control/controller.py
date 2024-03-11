"""Low level control logic for actuating the rudder and the sail."""

from abc import ABC, abstractmethod
from math import atan2, cos, pi, sin

import numpy as np

from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180


class ActuatorController(ABC):
    """Abstract class for rudder and sail actuation mechanisms.

    Attributes:
        `current_control_ang` (Scalar): Current control mechanism angle in degrees.
        `time_step` (Scalar): Time per iteration given in degrees/second.
        `max_angle_range` (tuple): Tuple of minimum to maximum control angle range
    """

    def __init__(
        self,
        current_control_ang: Scalar,
        time_step: Scalar,
        max_angle_range: tuple,
    ):
        """Initializes the class attributes. Note that this class cannot be directly instantiated.

        Args:
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in degrees/second.
            `max_angle_range` (tuple): Tuple of minimum to maximum control angle range
        """
        self.current_control_ang = current_control_ang
        self.time_step = time_step
        self.max_angle_range = max_angle_range

    @abstractmethod
    def update_state(self) -> bool:
        """Updates the controller position based on target heading angles

        Returns:
            False if target angle was not reached, else True if desired heading has been reached
        """

        pass

    @abstractmethod
    def compute_error_angle(self) -> Scalar:
        """Computes the corresponding feedback based on error and desired heading"""
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
        max_angle_range: tuple,
        rudder_speed: Scalar,
    ):
        """Initializes the class attributes.

        Args:
            `current_heading` (Scalar): Current boat heading direction in degrees.
            `desired_heading` (Scalar): Target boating heading direction in degrees.
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in degrees/second.
            `kp` (Scalar): Proportional constant when calculating error.
            `cp` (Scalar): Timeseries of error values computed over time.
            `max_angle_range` (tuple): Tuple of minimum to maximum control angle range in degrees
            `rudder_speed` (Scalar): Speed of controller change in degrees per second

        """
        super().__init__(current_control_ang, time_step, max_angle_range)

        self.current_heading = current_heading
        self.desired_heading = desired_heading
        self.kp = kp
        self.cp = cp
        self.max_angle_range = max_angle_range
        self.rudder_speed = rudder_speed
        self.running_error = 0.0  # keeps track of remaining error between desired and current

    def compute_error(
        self, desired: Scalar, current: Scalar
    ) -> Scalar:  # angle passed in as radians
        """Computes the error between desired and current heading
        implementation taken from: https://stackoverflow.com/a/2007279
        Angles are bound with the convention (-180, 180]

        Args:
            `desired` (Scalar): New desired heading in degrees
            `current` (Scalar): New current heading in degrees

        Returns:
            Scalar: The error between the given headings in radians.
        """
        current_bound, desired_bound = map(bound_to_180, (current, desired))
        desired_rad, current_rad = np.deg2rad(desired_bound), np.deg2rad(current_bound)

        error = atan2(sin(desired_rad - current_rad), cos(desired_rad - current_rad))

        return error

    def compute_error_angle(self) -> Scalar:
        """Computes the feedback angle from the given error (in radians)
        Calculations done using Raye's implementation
            https://github.com/UBCSailbot/raye-boat-controller/blob/master/python/tack_controller.py

        Returns:
            Scalar: Difference between current and target rudder angle in degrees.
        """
        error = self.compute_error(self.current_heading, self.desired_heading)

        if abs(error) > (pi / 4):  # limited between (-45, 45)
            raise ValueError("Heading_error must be between -pi/4 and pi/4")

        rudder_change = (self.kp * error) / (1 + (self.cp * error))

        if abs(rudder_change) > (pi / 2):  # greatest change is 90 degrees
            raise ValueError("Rudder_change cannot be greater than pi / 2")

        rudder_change_deg = np.rad2deg(rudder_change)
        self.running_error = rudder_change_deg

        return rudder_change_deg

    def compute_setpoint(self) -> Scalar:
        """Calculates the rudder setpoint angle. Only called at the very beginning once
        to determine the setpoint angle.

        Returns:
            The rudder setpoint angle in degrees
        """
        return self.current_control_ang + self.compute_error_angle()

    def reset_desired_heading(self, target_angle: Scalar) -> None:
        """Resets a new desired heading angle. Must recompute all errors and
        feedback angles again for new iteration loop.

        Args:
            `target_angle` (Scalar): New desired heading

        """
        self.desired_heading = target_angle

    def update_state(self) -> bool:
        """Updates the rudder angle iteratively towards the target rudder setpoint based
        on the rudder_speed and time step parameters

        Returns:
            False if target angle was not reached, else True if desired heading has been reached
        """

        if self.running_error > 0:
            change = self.rudder_speed * self.time_step
        elif self.running_error < 0:
            change = -self.rudder_speed * self.time_step
        else:
            return True

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
        max_angle_range: tuple,
        sail_control_speed: Scalar,
    ):
        """Initializes the class attributes.

        Args:
            `current_heading` (Scalar): Current boat heading direction in degrees.
            `desired_heading` (Scalar): Target boating heading direction in degrees.
            `current_control_ang` (Scalar): Current control mechanism angle in degrees.
            `time_step` (Scalar): Time per iteration given in degrees/second.
            `kp` (Scalar): Proportional constant when calculating error.
            `cp` (Scalar): Timeseries of error values computed over time.
            `max_angle_range` (tuple): Tuple of minimum to maximum control angle range

        """
        super().__init__(current_control_ang, time_step, max_angle_range)

        self.target_angle = target_angle
        self.kp = kp
        self.cp = cp
        self.sail_control_speed = sail_control_speed
        self.running_error = 0.0

    def compute_error_angle(self):
        """Computes the feedback angle between setpoint and current sail angles"""
        return self.target_angle - self.current_control_ang

    def reset_target_angle(self, new_target: Scalar) -> None:
        """Resets a new desired heading angle and clears all previous
        headings from current_control_ang list and running time for time_series

        Args:
            `target_angle` (Scalar): New desired heading

        """
        self.target_angle = new_target

    def update_state(self) -> bool:
        """Updates the sail angle iteratively towards the target sail setpoint

        Returns:
            False if target angle was not reached, else True if target angle has been reached
        """
        if self.running_error > 0:
            change = self.sail_control_speed * self.time_step
        elif self.running_error < 0:
            change = -self.sail_control_speed * self.time_step
        else:
            return True

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
