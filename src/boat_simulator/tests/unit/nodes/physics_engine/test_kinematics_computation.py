"""Tests classes and functions in boat_simulator/nodes/physics_engine/kinematics_computation.py"""

import math

import numpy as np
import pytest

from boat_simulator.common.angle_conventions import wrap_to_pi
from boat_simulator.common.constants import BOAT_PROPERTIES
from boat_simulator.common.conventions import Body, Force
from boat_simulator.common.types import Vec4
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics

MASS = BOAT_PROPERTIES.mass
I_X = BOAT_PROPERTIES.inertia.data[2, 2]
I_Z = BOAT_PROPERTIES.inertia.data[3, 3]
TIMESTEP = 0.1

# The generalized force [X, Y, K, N] carries both forces (surge/sway) and moments (roll/yaw).
ZERO_FORCE = Vec4[Force, Body].from_xypr(0.0, 0.0, 0.0, 0.0)


def make_kinematics(timestep: float = TIMESTEP) -> BoatKinematics:
    return BoatKinematics(timestep=timestep)


# --- construction ---------------------------------------------------------


def test_construction_stores_timestep_mass_and_inertia() -> None:
    kinematics = make_kinematics()

    assert kinematics.timestep == pytest.approx(TIMESTEP)
    assert kinematics.boat_mass == pytest.approx(MASS)
    np.testing.assert_array_equal(kinematics.inertia.data, BOAT_PROPERTIES.inertia.data)


def test_construction_inverts_inertia() -> None:
    kinematics = make_kinematics()

    expected_inverse = np.linalg.inv(BOAT_PROPERTIES.inertia.data)
    np.testing.assert_allclose(kinematics.inertia_inverse.data, expected_inverse)


def test_construction_initializes_zero_state() -> None:
    kinematics = make_kinematics()

    np.testing.assert_array_equal(kinematics.pose.data, np.zeros(4))
    np.testing.assert_array_equal(kinematics.nu.data, np.zeros(4))
    np.testing.assert_array_equal(kinematics.nu_dot.data, np.zeros(4))


# --- step(): zero input -----------------------------------------------------


def test_step_with_zero_force_leaves_state_at_rest() -> None:
    kinematics = make_kinematics()

    kinematics.step(kinematics.nu, ZERO_FORCE)

    np.testing.assert_array_equal(kinematics.nu_dot.data, np.zeros(4))
    np.testing.assert_array_equal(kinematics.nu.data, np.zeros(4))
    np.testing.assert_array_equal(kinematics.pose.data, np.zeros(4))


# --- step(): acceleration ---------------------------------------------------


@pytest.mark.parametrize(
    ("net_force", "expected_nu_dot"),
    [
        (Vec4.from_xypr(21.0, 0.0, 0.0, 0.0), [21.0 / MASS, 0.0, 0.0, 0.0]),
        (Vec4.from_xypr(0.0, 42.0, 0.0, 0.0), [0.0, 42.0 / MASS, 0.0, 0.0]),
        (Vec4.from_xypr(0.0, 0.0, 25.0, 0.0), [0.0, 0.0, 25.0 / I_X, 0.0]),
        (Vec4.from_xypr(0.0, 0.0, 0.0, 100.0), [0.0, 0.0, 0.0, 100.0 / I_Z]),
    ],
    ids=["surge-force", "sway-force", "roll-torque", "yaw-torque"],
)
def test_step_computes_acceleration_per_dof(net_force, expected_nu_dot) -> None:
    kinematics = make_kinematics()

    kinematics.step(kinematics.nu, net_force)

    np.testing.assert_allclose(kinematics.nu_dot.data, expected_nu_dot)


def test_step_does_not_cross_couple_force_and_torque_dofs() -> None:
    # With a diagonal generalized inertia matrix, the surge/sway components must not perturb
    # roll/yaw and the roll/yaw components must not perturb surge/sway.
    kinematics = make_kinematics()
    net_force = Vec4[Force, Body].from_xypr(10.0, -5.0, 3.0, -7.0)

    kinematics.step(kinematics.nu, net_force)

    expected = [10.0 / MASS, -5.0 / MASS, 3.0 / I_X, -7.0 / I_Z]
    np.testing.assert_allclose(kinematics.nu_dot.data, expected)


# --- step(): velocity integration --------------------------------------------


def test_step_integrates_velocity_from_acceleration() -> None:
    kinematics = make_kinematics()
    net_force = Vec4[Force, Body].from_xypr(21.0, 0.0, 0.0, 0.0)

    kinematics.step(kinematics.nu, net_force)

    expected_u = (21.0 / MASS) * TIMESTEP
    assert kinematics.nu.x == pytest.approx(expected_u)


def test_step_accumulates_velocity_over_multiple_steps() -> None:
    kinematics = make_kinematics()
    net_force = Vec4[Force, Body].from_xypr(21.0, 0.0, 0.0, 0.0)
    num_steps = 5

    for _ in range(num_steps):
        kinematics.step(kinematics.nu, net_force)

    # No drag is modeled here and pure surge produces no Coriolis coupling, so the
    # acceleration is constant across steps and the velocity grows linearly.
    assert kinematics.nu_dot.x == pytest.approx(21.0 / MASS)
    assert kinematics.nu.x == pytest.approx((21.0 / MASS) * TIMESTEP * num_steps)


# --- step(): pose integration via J(eta) -------------------------------------


def test_step_integrates_north_east_position_at_zero_heading() -> None:
    kinematics = make_kinematics()
    net_force = Vec4[Force, Body].from_xypr(21.0, 14.0, 0.0, 0.0)

    kinematics.step(kinematics.nu, net_force)

    # At psi = phi = 0, J(eta)'s rotation block is the identity, so eta_dot == nu (the
    # already-updated body velocity from this same step).
    updated_u = (21.0 / MASS) * TIMESTEP
    updated_v = (14.0 / MASS) * TIMESTEP
    assert kinematics.pose.x == pytest.approx(updated_u * TIMESTEP)
    assert kinematics.pose.y == pytest.approx(updated_v * TIMESTEP)


def test_step_rotates_body_velocity_into_ned_position_by_heading() -> None:
    kinematics = make_kinematics()
    psi = math.pi / 2
    kinematics.kinematics.pose = Vec4.from_xypr(0.0, 0.0, 0.0, psi)
    net_force = Vec4[Force, Body].from_xypr(21.0, 0.0, 0.0, 0.0)

    kinematics.step(kinematics.nu, net_force)

    updated_u = (21.0 / MASS) * TIMESTEP
    expected_north = updated_u * math.cos(psi) * TIMESTEP
    expected_east = updated_u * math.sin(psi) * TIMESTEP
    assert kinematics.pose.x == pytest.approx(expected_north, abs=1e-9)
    assert kinematics.pose.y == pytest.approx(expected_east, abs=1e-9)


def test_step_integrates_roll_and_yaw_rate_into_pose() -> None:
    kinematics = make_kinematics()
    net_force = Vec4[Force, Body].from_xypr(0.0, 0.0, 12.5, 40.0)

    kinematics.step(kinematics.nu, net_force)

    updated_p = (12.5 / I_X) * TIMESTEP
    updated_r = (40.0 / I_Z) * TIMESTEP
    # phi starts at 0, so psi_dot = r * cos(phi) == r here.
    assert kinematics.pose.p == pytest.approx(updated_p * TIMESTEP)
    assert kinematics.pose.r == pytest.approx(updated_r * TIMESTEP)


def test_step_scales_yaw_rate_by_cosine_of_roll() -> None:
    kinematics = make_kinematics()
    # Fix the boat at a nonzero roll angle with zero roll rate, and a nonzero yaw rate, to
    # isolate the psi_dot = r * cos(phi) term.
    roll_angle = math.pi / 3
    kinematics.kinematics.pose = Vec4.from_xypr(0.0, 0.0, roll_angle, 0.0)
    kinematics.kinematics.nu = Vec4.from_xypr(0.0, 0.0, 0.0, 2.0)

    kinematics.step(kinematics.nu, ZERO_FORCE)

    expected_psi = 2.0 * math.cos(roll_angle) * TIMESTEP
    assert kinematics.pose.r == pytest.approx(expected_psi)
    # Roll rate p is 0, so phi should be unaffected even though phi != 0.
    assert kinematics.pose.p == pytest.approx(roll_angle)


def test_step_uses_updated_velocity_for_position_integration() -> None:
    """Locks in the semi-implicit (symplectic) Euler scheme documented on `step`: eta_dot must be
    computed from nu *after* this step's acceleration has been integrated in, not the nu from
    before the step.
    """
    kinematics = make_kinematics()
    kinematics.kinematics.nu = Vec4.from_xypr(1.0, 0.0, 0.0, 0.0)
    net_force = Vec4[Force, Body].from_xypr(21.0, 0.0, 0.0, 0.0)

    kinematics.step(kinematics.nu, net_force)

    updated_u = 1.0 + (21.0 / MASS) * TIMESTEP
    semi_implicit_expected = updated_u * TIMESTEP
    explicit_euler_expected = 1.0 * TIMESTEP

    assert kinematics.pose.x == pytest.approx(semi_implicit_expected)
    assert kinematics.pose.x != pytest.approx(explicit_euler_expected)


# --- step(): angle wrapping ---------------------------------------------------


def test_step_wraps_yaw_to_within_pi() -> None:
    kinematics = make_kinematics()
    kinematics.kinematics.pose = Vec4.from_xypr(0.0, 0.0, 0.0, math.pi - 0.05)
    kinematics.kinematics.nu = Vec4.from_xypr(0.0, 0.0, 0.0, 1.0)

    kinematics.step(kinematics.nu, ZERO_FORCE)

    expected_psi = wrap_to_pi((math.pi - 0.05) + 1.0 * TIMESTEP)
    assert -math.pi <= kinematics.pose.r < math.pi
    assert kinematics.pose.r == pytest.approx(expected_psi)


def test_step_wraps_roll_to_within_pi() -> None:
    kinematics = make_kinematics()
    kinematics.kinematics.pose = Vec4.from_xypr(0.0, 0.0, -math.pi + 0.02, 0.0)
    kinematics.kinematics.nu = Vec4.from_xypr(0.0, 0.0, -1.0, 0.0)

    kinematics.step(kinematics.nu, ZERO_FORCE)

    expected_phi = wrap_to_pi((-math.pi + 0.02) + (-1.0) * TIMESTEP)
    assert -math.pi <= kinematics.pose.p < math.pi
    assert kinematics.pose.p == pytest.approx(expected_phi)


def test_step_does_not_wrap_north_east_position() -> None:
    kinematics = make_kinematics()
    kinematics.kinematics.pose = Vec4.from_xypr(1000.0, -1000.0, 0.0, 0.0)
    kinematics.kinematics.nu = Vec4.from_xypr(5.0, -5.0, 0.0, 0.0)

    kinematics.step(kinematics.nu, ZERO_FORCE)

    assert kinematics.pose.x == pytest.approx(1000.0 + 5.0 * TIMESTEP)
    assert kinematics.pose.y == pytest.approx(-1000.0 - 5.0 * TIMESTEP)
