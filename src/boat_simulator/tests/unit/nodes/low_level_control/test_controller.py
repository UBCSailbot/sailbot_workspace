"""Tests classes and functions in boat_simulator/nodes/controller.py"""

from math import atan2, copysign, cos, sin

import numpy as np
import pytest

from boat_simulator.nodes.low_level_control.controller import (
    RudderController,
    SailController,
)


@pytest.fixture(
    params=[
        (60.3, 45, 0, 0.5, 0.7, 0.34, 2),
        (-70.2, 30, 25, 0.5, 0.7, 0.34, 0.5),
        (30.8, 20.4, 25, 1, 0.7, 0.34, 2),
        (-24, -24, 9.2, 0.5, 0.7, 0.34, 2),
        (90, -90, 30, 2, 0.7, 0.34, 1.5),
        (50.488, -36.78, 30, 0.5, 0.7, 0.34, 1),
    ]
)
def rudder_controller(request):
    # Initialize the RudderController object with parameters from the fixture
    current_heading, desired_heading, current_control_ang, time_step, kp, cp, control_speed = (
        request.param
    )
    return RudderController(
        current_heading=current_heading,
        desired_heading=desired_heading,
        current_control_ang=current_control_ang,
        time_step=time_step,
        kp=kp,
        cp=cp,
        control_speed=control_speed,
    )


def test_compute_error(rudder_controller):
    # Test compute_error method of RudderController
    error = rudder_controller.compute_error()
    desired_rad = np.deg2rad(rudder_controller.desired_heading)
    current_rad = np.deg2rad(rudder_controller.current_heading)
    exp_error = atan2(sin(desired_rad - current_rad), cos(desired_rad - current_rad))
    assert np.equal(error, exp_error)


def test_compute_setpoint(rudder_controller):
    # Test compute_setpoint method of RudderController
    setpoint = rudder_controller.compute_setpoint()
    heading_error = rudder_controller.compute_error()  # heading_error in radians

    rudder_setpoint = (rudder_controller.kp * heading_error) / (
        1 + (rudder_controller.cp * abs(heading_error))
    )

    rudder_setpoint = min(
        max(rudder_setpoint, np.deg2rad(rudder_controller.max_angle_range[0])),
        np.deg2rad(rudder_controller.max_angle_range[1]),
    )

    rudder_setpoint_deg = np.rad2deg(rudder_setpoint)  # in degrees
    running_error = rudder_setpoint_deg - rudder_controller.current_control_ang  # in degrees
    assert np.equal(setpoint, rudder_setpoint_deg)
    assert np.equal(running_error, rudder_controller.running_error)


def test_reset_setpoint(rudder_controller):
    # Test reset_setpoint method of RudderController
    new_desired_heading = 60
    new_current_heading = 30
    expected = rudder_controller.reset_setpoint(new_desired_heading, new_current_heading)
    rudder_controller.change_desired_heading(new_desired_heading)
    calculated = rudder_controller.compute_setpoint()
    np.equal(calculated, expected)


# testing for one iteration only
def test_update_state(rudder_controller):
    current_control = rudder_controller.current_control_ang
    rudder_controller.compute_setpoint()
    assert rudder_controller.update_state() == (abs(rudder_controller.running_error) == 0)
    assert np.equal(
        current_control
        + (
            copysign(
                rudder_controller.control_speed * rudder_controller.time_step,
                rudder_controller.running_error,
            )
        ),
        rudder_controller.current_control_ang,
    )


def test_update_state_continuous(rudder_controller):
    rudder_controller.compute_setpoint()
    progress = False
    counter = 0
    if np.isclose(rudder_controller.running_error, 0, 0.1):
        progress = True
    else:
        while not np.isclose(rudder_controller.running_error, 0, 0.1):
            counter += 1
            rudder_controller.update_state()
            if rudder_controller.running_error == 0:
                progress = True
                break
            if counter > 1000:
                break
    assert progress
    assert np.equal(rudder_controller.current_control_ang, rudder_controller.setpoint)


def test_update_state_reset(rudder_controller):
    rudder_controller.compute_setpoint()
    counter = 0
    while not np.isclose(rudder_controller.running_error, 0, 0.1):
        counter += 1
        rudder_controller.update_state()
        if rudder_controller.running_error == 0:
            break
        if counter > 1000:
            break
    rudder_controller.reset_setpoint(-30, 10)
    reset_progress = False
    if np.isclose(rudder_controller.running_error, 0, 0.1):
        reset_progress = True
    else:
        while not np.isclose(rudder_controller.running_error, 0, 0.1):
            counter += 1
            rudder_controller.update_state()
            if rudder_controller.running_error == 0:
                reset_progress = True
                break
            if counter > 1000:
                break
    assert reset_progress
    assert np.isclose(rudder_controller.current_control_ang, rudder_controller.setpoint, 0.1)


@pytest.fixture(
    params=[
        (45, 0.9, 0.5, 0.7, 0.34, 2),  # Test case 1
        (-45.2, 15, 1, 0.7, 0.34, 1.5),
        (0, -60, 2, 0.7, 0.34, 0.5),
        (70.2, -70.5, 0.5, 0.7, 0.34, 1),
        (24, 24, 1, 0.7, 0.34, 2),
    ]
)
def sail_controller(request):
    # Initialize the SailController object with parameters from the fixture
    target_angle, current_control_ang, time_step, kp, cp, control_speed = request.param
    return SailController(
        target_angle=target_angle,
        current_control_ang=current_control_ang,
        time_step=time_step,
        kp=kp,
        cp=cp,
        control_speed=control_speed,
    )


def test_compute_error_1(sail_controller):
    # Test compute_error method of SailController
    error = sail_controller.compute_error()
    exp_error = sail_controller.target_angle - sail_controller.current_control_ang
    assert np.equal(error, exp_error)


def test_reset_setpoint_1(sail_controller):
    # Test reset_setpoint method of SailController
    new_target = 90
    sail_controller.reset_setpoint(new_target)
    assert np.equal(new_target, sail_controller.target_angle)


def test_update_state_1(sail_controller):
    current_control = sail_controller.current_control_ang
    iteration_speed = copysign(
        sail_controller.control_speed * sail_controller.time_step, sail_controller.running_error
    )
    assert sail_controller.update_state() == (abs(sail_controller.running_error) == 0)
    assert (
        np.isclose(sail_controller.current_control_ang, current_control + iteration_speed, 0.1)
        or sail_controller.running_error == 0
    )


def test_update_state_continuous_1(sail_controller):
    sail_controller.compute_error()
    progress = False
    counter = 0
    if np.isclose(sail_controller.running_error, 0, 0.1):
        progress = True
    else:
        while not np.isclose(sail_controller.running_error, 0, 0.1):
            counter += 1
            sail_controller.update_state()
            if sail_controller.running_error == 0:
                progress = True
                break
            if counter > 1000:
                break
    assert progress
