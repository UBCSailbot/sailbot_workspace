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
            (-45.5, 360.7, -20, 2, 0.7, 0.34, (45, -45), 0.5),
            (-180, 180, 0, 1.5, 0.7, 0.34, (45, -45), 1),
            (0, 70, 7.5, 0.5, 0.7, 0.34, (45, -45), 2),
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
        max_angle_range: tuple,
        rudder_speed: Scalar,
    ):

        rudder_controller = RudderController(
            current_heading,
            desired_heading,
            current_control_ang,
            time_step,
            kp,
            cp,
            max_angle_range,
            rudder_speed,
        )
        error = rudder_controller.compute_error(desired_heading, current_heading)
        desired_rad = np.deg2rad(bound_to_180(desired_heading))
        current_rad = np.deg2rad(bound_to_180(current_heading))
        expected_error = atan2(sin(desired_rad - current_rad), cos(desired_rad - current_rad))
        assert np.equal(expected_error, error)

    @pytest.mark.parametrize(
        "current_heading, desired_heading, current_control_ang,time_step, kp, cp, max_angle_range",
        [
            (60, 45, 10, 0.5, 0.7, 0.34, (45, -45)),
        ],
    )
    def test_compute_feedback_angle(
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

        rudder_controller = RudderController(
            current_heading,
            desired_heading,
            current_control_ang,
            time_step,
            kp,
            cp,
            max_angle_range,
            rudder_speed,
        )
        feedback_angle = rudder_controller.compute_feedback_angle()
        error = rudder_controller.compute_error(desired_heading, current_heading)
        expected_angle = (rudder_controller.kp * error) / (1 + (rudder_controller.cp * error))
        assert np.equal(feedback_angle, expected_angle)

    @pytest.mark.parametrize(
        "current_heading, desired_heading, current_control_ang,time_step, kp, cp, max_angle_range",
        [
            (60, 45, 10, 0.5, 0.7, 0.34, (45, -45)),
        ],
    )
    def test_compute_setpoint(
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

        rudder_controller = RudderController(
            current_heading,
            desired_heading,
            current_control_ang,
            time_step,
            kp,
            cp,
            max_angle_range,
            rudder_speed,
        )
        setpoint = rudder_controller.compute_setpoint()
        expected_setpoint = (
            rudder_controller.current_control_ang + rudder_controller.compute_feedback_angle()
        )
        assert np.equal(setpoint, expected_setpoint)

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
        max_angle_range: tuple,
        rudder_speed: Scalar,
    ):

        rudder_controller = RudderController(
            current_heading,
            desired_heading,
            current_control_ang,
            time_step,
            kp,
            cp,
            max_angle_range,
            rudder_speed,
        )
        rudder_controller.update_state()
        assert np.equal(rudder_controller.current_control_ang, 12)
