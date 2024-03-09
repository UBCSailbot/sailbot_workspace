"""Test the functions in boat_simulator/common/utils.py"""

import math

import numpy as np
import pytest

from boat_simulator.common import utils


@pytest.mark.parametrize(
    "test_input, expected_output",
    [
        (math.pi, 180),
        (math.pi / 2, 90),
        (math.pi / 4, 45),
        (0, 0),
        (math.pi / 12, 180 / 12),
        (-math.pi / 4.5, -180 / 4.5),
    ],
)
def test_rad_to_degrees(test_input, expected_output):
    actual_output = utils.rad_to_degrees(test_input)
    assert math.isclose(actual_output, expected_output)


@pytest.mark.parametrize(
    "test_input, expected_output",
    [
        (180, math.pi),
        (90, math.pi / 2),
        (45, math.pi / 4),
        (0, 0),
        (180 / 12, math.pi / 12),
        (-180 / 4.5, -math.pi / 4.5),
    ],
)
def test_degrees_to_rad(test_input, expected_output):
    actual_output = utils.degrees_to_rad(test_input)
    assert math.isclose(actual_output, expected_output)


@pytest.mark.parametrize(
    "angle, isDegrees, expected_output",
    [
        # Degree tests
        (0, True, 0),
        (270, True, -90),
        (-450, True, -90),
        (360, True, 0),
        (
            np.array([540, -540, 899, -899, 5, -30]),
            True,
            np.array([-180, -180, 179, -179, 5, -30]),
        ),
        # Radian tests
        (0, False, 0),
        (2 * math.pi, False, 0),
        (3 / 2 * math.pi, False, -0.5 * math.pi),
        (-2.5 * math.pi, False, -0.5 * math.pi),
        (np.array([3 * math.pi, -3 * math.pi]), False, np.array([-math.pi, -math.pi])),
        (
            np.array([4.44 * math.pi, -5.68 * math.pi]),
            False,
            np.array([0.44 * math.pi, 0.32 * math.pi]),
        ),
        (
            np.array([1 / 36 * math.pi, -0.334 * math.pi]),
            False,
            np.array([1 / 36 * math.pi, -0.334 * math.pi]),
        ),
    ],
)
def test_bound_to_180(angle, isDegrees, expected_output):
    actual_output = utils.bound_to_180(angle, isDegrees)
    assert np.isclose(actual_output, expected_output).all()


@pytest.mark.parametrize(
    "angle, isDegrees, expected_output",
    [
        # Degree tests
        (0, True, 0),
        (360, True, 0),
        (-270, True, 90),
        (750.2, True, 30.2),
        (
            np.array([754.4, -540, 899, 719.99, 5, -30.74]),
            True,
            np.array([34.4, 180, 179, 359.99, 5, 329.26]),
        ),
        # Radian tests
        (0, False, 0),
        (2 * math.pi, False, 0),
        (1.5 * math.pi, False, 1.5 * math.pi),
        (-2.5 * math.pi, False, 1.5 * math.pi),
        (np.array([3 * math.pi, -3 * math.pi]), False, np.array([math.pi, math.pi])),
        (
            np.array([4.44 * math.pi, -5.68 * math.pi]),
            False,
            np.array([0.44 * math.pi, 0.32 * math.pi]),
        ),
        (
            np.array([1 / 36 * math.pi, -0.334 * math.pi]),
            False,
            np.array([1 / 36 * math.pi, 1.666 * math.pi]),
        ),
    ],
)
def test_bound_to_360(angle, isDegrees, expected_output):
    actual_output = utils.bound_to_360(angle, isDegrees)
    assert np.isclose(actual_output, expected_output).all()
