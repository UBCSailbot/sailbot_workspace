import math

import numpy as np
import pytest

from boat_simulator.nodes.physics_engine.fluid_forces import MediumForceComputation


@pytest.fixture
def medium_force_setup():
    lift_coefficients = np.array([[0, 0], [5, 0.57], [10, 1.10], [15, 1.39], [20, 1.08]])
    drag_coefficients = np.array([[0, 0.013], [5, 0.047], [10, 0.144], [15, 0.279], [20, 0.298]])
    areas = 9.0
    fluid_density = 1.225

    computation = MediumForceComputation(
        lift_coefficients, drag_coefficients, areas, fluid_density
    )
    return computation


def test_initialization(medium_force_setup):
    assert isinstance(medium_force_setup.lift_coefficients, np.ndarray)
    assert isinstance(medium_force_setup.drag_coefficients, np.ndarray)
    assert isinstance(medium_force_setup.areas, (int, float))
    assert isinstance(medium_force_setup.fluid_density, (int, float))


@pytest.mark.parametrize(
    "apparent_velocity, orientation, expected_angle",
    [
        # Test zero apparent velocity with various orientations,
        # including edge cases and normalization
        (np.array([0, 0]), 0, 0),
        (np.array([0, 0]), 45, 45),
        (np.array([0, 0]), 90, 90),
        (np.array([0, 0]), 180, -180),  # Normalized to -180
        (np.array([0, 0]), 270, -90),  # Normalized to -90
        (np.array([0, 0]), 360, 0),
        (np.array([0, 0]), -45, -45),  # Test negative orientation
        (np.array([0, 0]), 405, 45),  # Orientation beyond 360
        (np.array([0, 0]), -405, -45),  # Orientation below -360
        # Test non-zero apparent velocity for comprehensive angle of attack calculations
        (np.array([1, 0]), 0, 0),
        (np.array([0, 1]), 0, 90),
        (np.array([-1, 0]), 0, -180),
        (np.array([0, -1]), 0, -90),
        (np.array([1, 1]), 45, 0),
        (np.array([-1, -1]), 135, 90),
        # Edge cases where orientation and velocity directions are opposite or identical
        (np.array([1, 0]), 180, -180),
        (np.array([-1, 0]), 180, 0),
        (np.array([0, 1]), 270, -180),
        (np.array([0, -1]), 90, -180),
        # Additional tests with non-unit vectors
        (np.array([2, 0]), 0, 0),  # Horizontal vector, twice the unit length
        (np.array([0, 2]), 0, 90),  # Vertical vector, twice the unit length
        (np.array([-2, 0]), 0, -180),  # Left horizontal, twice the unit length
        (np.array([3, 4]), 0, np.rad2deg(np.arctan2(4, 3))),  # 3-4-5 triangle vector
        (np.array([5, 5]), 45, 0),  # Diagonal upward, aligned with orientation
    ],
)
def test_calculate_attack_angle(
    apparent_velocity, orientation, expected_angle, medium_force_setup
):
    attack_angle = medium_force_setup.calculate_attack_angle(apparent_velocity, orientation)
    assert np.isclose(
        attack_angle, expected_angle, atol=1e-7
    ), f"Expected {expected_angle}, got {attack_angle}"


# Test taken from https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/foilsimstudent/
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
            np.array([44 * math.cos(np.deg2rad(20)), 44 * math.sin(np.deg2rad(20))]),
        ),
    ],
)
def test_compute_forces(
    medium_force_setup, orientation, expected_lift, expected_drag, apparent_velocity
):
    lift_force, drag_force = medium_force_setup.compute(apparent_velocity, orientation)
    assert np.isclose(
        np.linalg.norm(lift_force), expected_lift, rtol=0.05
    ), f"Expected {expected_lift}, got {np.linalg.norm(lift_force)}"
    assert np.isclose(
        np.linalg.norm(drag_force), expected_drag, rtol=0.05
    ), f"Expected {expected_drag}, got {np.linalg.norm(drag_force)}"
