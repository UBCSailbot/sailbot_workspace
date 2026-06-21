import math

import numpy as np
import pytest

from boat_simulator.common.frames import (
    Body,
    Force,
    Heading,
    NED,
    Position,
    SteeringAngle,
    Vec2,
    Vec3,
    clamp,
    saturated_steering_angle,
    wrap_to_pi,
)


def test_vec3_construction_access_and_arithmetic() -> None:
    first = Vec3[Position, NED].from_xyz(1, 2, 3)
    second = Vec3[Position, NED].from_xyz(4, 5, 6)

    assert (first.x, first.y, first.z) == (1.0, 2.0, 3.0)
    np.testing.assert_array_equal((first + second).data, [5, 7, 9])
    np.testing.assert_array_equal((second - first).data, [3, 3, 3])
    np.testing.assert_array_equal((2 * first).data, [2, 4, 6])


def test_vec2_construction_access_and_arithmetic() -> None:
    first = Vec2[Force, Body].from_xy(1, 2)
    second = Vec2[Force, Body].from_xy(3, 4)

    assert (first.x, first.y) == (1.0, 2.0)
    np.testing.assert_array_equal((first + second).data, [4, 6])
    np.testing.assert_array_equal((second - first).data, [2, 2])
    np.testing.assert_array_equal((first * 3).data, [3, 6])


@pytest.mark.parametrize(
    ("vector_type", "data", "expected_shape"),
    [(Vec2, np.zeros(3), "(2,)"), (Vec3, np.zeros(2), "(3,)")],
)
def test_vectors_reject_incorrect_shapes(vector_type, data, expected_shape) -> None:
    with pytest.raises(ValueError, match=expected_shape):
        vector_type(data)


def test_vectors_own_immutable_float_data() -> None:
    source = np.array([1, 2, 3])
    vector = Vec3[Position, NED](source)
    source[0] = 99

    assert vector.data.dtype == np.float64
    assert vector.x == 1.0
    with pytest.raises(ValueError, match="read-only"):
        vector.data[0] = 99


def test_vectors_have_value_equality_and_hashing() -> None:
    first = Vec3[Position, NED].from_xyz(1, 2, 3)
    same = Vec3[Position, NED].from_xyz(1, 2, 3)
    different = Vec3[Position, NED].from_xyz(1, 2, 4)

    assert first == same
    assert first != different
    assert hash(first) == hash(same)


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0.0, 0.0), (math.pi, -math.pi), (-math.pi, -math.pi), (3 * math.pi, -math.pi)],
)
def test_wrap_to_pi(angle: float, expected: float) -> None:
    assert wrap_to_pi(angle) == pytest.approx(expected)


@pytest.mark.parametrize("angle", [math.inf, -math.inf, math.nan])
def test_angles_reject_non_finite_values(angle: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        Heading(angle)
    with pytest.raises(ValueError, match="finite"):
        SteeringAngle(angle)
    with pytest.raises(ValueError, match="finite"):
        saturated_steering_angle(angle)


def test_heading_normalizes_and_converts_units() -> None:
    heading = Heading.from_degrees(270)
    assert heading.radians == pytest.approx(-math.pi / 2)
    assert heading.degrees == pytest.approx(-90)


def test_steering_angle_validates_and_saturates() -> None:
    assert SteeringAngle.from_degrees(30).degrees == pytest.approx(30)
    with pytest.raises(ValueError, match=r"\[-30 deg, 30 deg\]"):
        SteeringAngle.from_degrees(31)

    assert saturated_steering_angle(math.radians(45)).degrees == pytest.approx(30)
    assert saturated_steering_angle(math.radians(-45)).degrees == pytest.approx(-30)


def test_clamp_rejects_reversed_bounds() -> None:
    assert clamp(2, 0, 1) == 1
    with pytest.raises(ValueError, match="exceeds"):
        clamp(0, 1, -1)
