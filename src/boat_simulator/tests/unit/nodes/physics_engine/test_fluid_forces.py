import math

import numpy as np
import pytest

from boat_simulator.nodes.physics_engine.fluid_forces import MediumForceComputation


@pytest.fixture
def medium_force_setup():
    lift_coefficients = np.array([[0, 0], [5, 0.57], [10, 1.10], [15, 1.39], [20, 1.08]])
    drag_coefficients = np.array([[0, 0.013], [5, 0.047], [10, 0.144], [15, 0.279], [20, 0.298]])
    areas = np.array([[0, 9.0], [45, 9.0], [90, 9.0], [135, 9.0], [180, 9.0]])
    fluid_density = 1.225

    computation = MediumForceComputation(
        lift_coefficients, drag_coefficients, areas, fluid_density
    )
    return computation


def test_initialization(medium_force_setup):
    assert isinstance(medium_force_setup.lift_coefficients, np.ndarray)
    assert isinstance(medium_force_setup.drag_coefficients, np.ndarray)
    assert isinstance(medium_force_setup.areas, np.ndarray)
    assert isinstance(medium_force_setup.fluid_density, (int, float))


@pytest.mark.parametrize(
    "apparent_velocity, orientation, expected_angle",
    [
        # Tests if apparent velocity is 0
        (np.array([0, 0]), 0, 0),
        (np.array([0, 0]), 45, 45),
        (np.array([0, 0]), 90, 90),
        (np.array([0, 0]), 180, 180),
        (np.array([0, 0]), 270, 90),
        (np.array([0, 0]), 360, 0),
        (np.array([0, 0]), 450, 90),
        # Tests if apparent velocity is not 0
        (np.array([1, 0]), 0, 0),
        (np.array([0, 1]), 0, 90),
        (np.array([-1, 0]), 0, 180),
        (np.array([0, -1]), 0, 90),
        (np.array([1, 1]), 0, 45),
        (np.array([1, -1]), 0, 45),
        (np.array([-1, 1]), 0, 135),
        (np.array([-1, -1]), 0, 135),
        # Tests for apparent velocity other than unit vectors
        (np.array([2, 0]), 0, 0),
        (np.array([0, 2]), 0, 90),
        (np.array([-2, 0]), 0, 180),
        (np.array([0, -2]), 0, 90),
        (np.array([2, 2]), 0, 45),
        (np.array([2, -2]), 0, 45),
        (np.array([-2, 2]), 0, 135),
        (np.array([-2, -2]), 0, 135),
        # Tests for orientation other than 0
        (np.array([1, 0]), 45, 45),
        (np.array([0, 1]), 45, 45),
        (np.array([-1, 0]), 45, 135),
        (np.array([0, -1]), 45, 135),
        (np.array([1, 1]), 45, 0),
        (np.array([1, -1]), 45, 90),
        (np.array([-1, 1]), 45, 90),
        (np.array([-1, -1]), 45, 180),
    ],
)
def test_calculate_attack_angle(
    apparent_velocity, orientation, expected_angle, medium_force_setup
):
    attack_angle = medium_force_setup.calculate_attack_angle(apparent_velocity, orientation)
    assert attack_angle == expected_angle


@pytest.mark.parametrize(
    "orientation, expected_lift, expected_drag, apparent_velocity",
    [
        # Tests for attack angle 0
        (0, 0, 140, np.array([44 * math.cos(0), 44 * math.sin(0)])),
        # # # Tests for attack angle 5
        (0, 6262, 509, np.array([44 * math.cos(np.deg2rad(5)), 44 * math.sin(np.deg2rad(5))])),
        # # Tests for attack angle 10
        (0, 11934, 1568, np.array([44 * math.cos(np.deg2rad(10)), 44 * math.sin(np.deg2rad(10))])),
        # # Tests for attack angle 15
        (0, 15162, 3035, np.array([44 * math.cos(np.deg2rad(15)), 44 * math.sin(np.deg2rad(15))])),
        # Tests for attack angle 20
        (
            0,
            11768,
            3249,
            np.array([44 * math.cos(np.deg2rad(20)), -44 * math.sin(np.deg2rad(20))]),
        ),
    ],
)
def test_compute_forces(
    medium_force_setup, orientation, expected_lift, expected_drag, apparent_velocity
):
    lift_force, drag_force = medium_force_setup.compute(apparent_velocity, orientation)

    def calculate_magnitude(force):
        return np.sqrt(force[0] ** 2 + force[1] ** 2)

    print(lift_force, drag_force)
    print(calculate_magnitude(lift_force), calculate_magnitude(drag_force))
    assert np.isclose(calculate_magnitude(lift_force), expected_lift, rtol=0.05)
    assert np.isclose(calculate_magnitude(drag_force), expected_drag, rtol=0.05)
