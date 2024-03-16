"""Tests classes and functions in boat_simulator/nodes/controller.py"""

from math import atan2, cos, sin

import numpy as np
import pytest

from boat_simulator.common.types import Scalar
from boat_simulator.common.utils import bound_to_180
from boat_simulator.nodes.low_level_control.controller import RudderController


class TestRudderController:
    @pytest.mark.parametrize(
        "current_heading, desired_heading, current_control_ang, \
            time_step, kp, cp, max_angle_range, rudder_speed",
        [
            (60, 45, 10.2, 0.5, 0.7, 0.34, (45, -45), 2),
            (180, 10, 4.7, 1, 0.7, 0.34, (45, -45), 1.5),
            (-45.5, 360.7, -20.5, 2, 0.7, 0.34, (45, -45), 0.5),
            (-180, 180, -45, 1.5, 0.7, 0.34, (45, -45), 1),
            (0, 70, 45, 0.5, 0.7, 0.34, (45, -45), 2),
        ],
    )
    def test_compute_error(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        current_control_ang: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
        control_speed: Scalar,
    ):

        rudder_controller = RudderController(
            current_heading,
            desired_heading,
            current_control_ang,
            time_step,
            kp,
            cp,
            control_speed,
        )
        error = rudder_controller.compute_error()
        desired_rad = np.deg2rad(bound_to_180(desired_heading))
        current_rad = np.deg2rad(bound_to_180(current_heading))
        expected_error = atan2(sin(desired_rad - current_rad), cos(desired_rad - current_rad))
        assert np.equal(expected_error, error)

    @pytest.mark.parametrize(
        "current_heading, desired_heading, \
            current_control_ang,time_step, kp, cp, max_angle_range, rudder_speed",
        [
            (60, 45, 10.2, 0.5, 0.7, 0.34, (45, -45), 2),
            (180, 10, 4.7, 1, 0.7, 0.34, (45, -45), 1.5),
            (-45.5, 360.7, -20.5, 2, 0.7, 0.34, (45, -45), 0.5),
            (-180, 180, -45, 1.5, 0.7, 0.34, (45, -45), 1),
            (0, 70, 45, 0.5, 0.7, 0.34, (45, -45), 2),
        ],
    )
    def test_compute_error_angle(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        current_control_ang: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
        control_speed: Scalar,
    ):

        rudder_controller = RudderController(
            current_heading, desired_heading, current_control_ang, time_step, kp, cp, control_speed
        )
        feedback_angle = rudder_controller.compute_setpoint()
        error = rudder_controller.compute_error()
        expected_angle = (rudder_controller.kp * error) / (1 + (rudder_controller.cp * error))
        assert np.equal(feedback_angle, np.rad2deg(expected_angle))

    @pytest.mark.parametrize(
        "current_heading, desired_heading, current_control_ang,time_step, kp, cp, max_angle_range",
        [
            (60, 45, 10, 0.5, 0.7, 0.34, (45, -45)),
        ],
    )
    def test_update_state(
        self,
        current_heading: Scalar,
        desired_heading: Scalar,
        current_control_ang: Scalar,
        time_step: Scalar,
        kp: Scalar,
        cp: Scalar,
        control_speed: Scalar,
    ):

        rudder_controller = RudderController(
            current_heading,
            desired_heading,
            current_control_ang,
            time_step,
            kp,
            cp,
            control_speed,
        )
        rudder_controller.update_state()
        assert np.equal(rudder_controller.current_control_ang, 12)
