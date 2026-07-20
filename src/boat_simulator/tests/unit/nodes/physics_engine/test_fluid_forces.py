"""Physics-invariant tests for boat_simulator/nodes/physics_engine/fluid_forces.py.

These tests assert properties that must hold no matter what the parameter values are, so
they catch whole classes of physics transcription errors (sign flips, non-skew Coriolis
matrices, energy-injecting "drag") without requiring the reference derivations:

1. Coriolis matrices (C_A, C_RB) are skew-symmetric — they redirect momentum but can
   never do work on the boat (nuᵀ·C·nu == 0).
2. The damping matrix D is positive semi-definite — damping only removes energy.
3. The total mass matrix M_RB + M_A is positive-definite — a negative or singular
   generalized mass produces instant divergence.
4. Rest stays at rest — zero state and zero input produce (near-)zero force.
5. Dissipative surfaces oppose motion — hull drag power is never positive.
6. The master energy test — coasting with no wind, current, or actuation, total
   mechanical energy must never increase on any timestep. Any term that injects
   energy (the class of bug that makes the sim explode) fails this immediately.
"""

import numpy as np
import pytest

from boat_simulator.common.constants import (
    BM_METACENTRIC_RADIUS,
    BOAT_PROPERTIES,
    DECK_EDGE_IMMERSION_ANGLE_RAD,
    DISPLACED_VOLUME,
    EARTH_GRAVITY,
    METACENTRIC_HEIGHT,
    WATER_DENSITY,
)
from boat_simulator.common.hydrostatics import restoring_potential_energy
from boat_simulator.common.types import Vec4
from boat_simulator.nodes.physics_engine.fluid_forces import (
    HydroDynamicsForceComputation,
)
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinetics_computation import (
    TotalForceComputation,
)

M_TOTAL = BOAT_PROPERTIES.inertia.data + BOAT_PROPERTIES.M_A.data


def random_velocities(count: int = 50, seed: int = 0) -> np.ndarray:
    return np.random.default_rng(seed).uniform(-5.0, 5.0, size=(count, 4))


# --- Coriolis matrices do no work (skew-symmetry) -----------------------------


def test_added_mass_coriolis_matrix_is_skew_symmetric() -> None:
    hydro = HydroDynamicsForceComputation()

    for nu in random_velocities():
        c_a = hydro.added_mass_coriolis_matrix(Vec4(nu))
        np.testing.assert_allclose(c_a, -c_a.T, atol=1e-12)


def test_added_mass_coriolis_matrix_does_no_work() -> None:
    hydro = HydroDynamicsForceComputation()

    for nu in random_velocities(seed=1):
        c_a = hydro.added_mass_coriolis_matrix(Vec4(nu))
        assert abs(nu @ c_a @ nu) == pytest.approx(0.0, abs=1e-9)


def test_rigid_body_coriolis_matrix_is_skew_symmetric() -> None:
    kinematics = BoatKinematics(timestep=0.1)

    for nu in random_velocities(seed=2):
        c_rb = kinematics.rigid_body_coriolis_matrix(Vec4(nu))
        np.testing.assert_allclose(c_rb, -c_rb.T, atol=1e-12)


# --- Damping only removes energy ----------------------------------------------


def test_damping_matrix_is_positive_semi_definite() -> None:
    d_sym = 0.5 * (BOAT_PROPERTIES.D.data + BOAT_PROPERTIES.D.data.T)

    assert np.all(np.linalg.eigvalsh(d_sym) >= 0.0)
    for nu in random_velocities(seed=3):
        assert nu @ BOAT_PROPERTIES.D.data @ nu >= 0.0


# --- The mass matrix is a valid generalized mass -------------------------------


def test_total_mass_matrix_is_positive_definite() -> None:
    m_sym = 0.5 * (M_TOTAL + M_TOTAL.T)

    assert np.all(np.linalg.eigvalsh(m_sym) > 0.0)


# --- Rest stays at rest ---------------------------------------------------------


def test_rest_stays_at_rest() -> None:
    kinematics = BoatKinematics(timestep=0.1)
    forces = TotalForceComputation()

    for _ in range(10):
        tau = forces.compute_total_force(
            kinematics,
            true_wind_speed_mps=0.0,
            true_wind_bearing_rad=0.0,
            ocean_current_speed_mps=0.0,
            ocean_current_bearing_rad=0.0,
            delta_r_rad=0.0,
            delta_tab_rad=0.0,
        )

        np.testing.assert_allclose(tau.data, np.zeros(4), atol=1e-9)
        kinematics.step(kinematics.nu, tau)
        np.testing.assert_allclose(kinematics.nu.data, np.zeros(4), atol=1e-9)


# --- Dissipative surfaces oppose motion -----------------------------------------


def test_hull_drag_power_is_never_positive() -> None:
    hydro = HydroDynamicsForceComputation()

    for nu in random_velocities(seed=4):
        force = hydro.hull_force(Vec4(nu), roll_rad=0.0)
        # Power delivered to the boat by a purely dissipative surface must be <= 0.
        power = force.data @ nu
        assert power <= 1e-9, f"hull injects energy: nu={nu} force={force.data}"


@pytest.mark.xfail(
    reason=(
        "keel_force's u_k=-u / orientation_deg=0 convention pins the attack angle near "
        "+-180 deg for any forward-moving state, outside the lift/drag tables' +-90 deg "
        "range, so the keel is silent (zero force) through ordinary forward sailing. It "
        "only re-engages once u_k=-u crosses back into (-90, 90) deg, i.e. reversed/near- "
        "zero surge combined with large roll/yaw rates, where this test finds it can "
        "inject energy. Needs the same orientation-tracking fix the wingsail already has "
        "(solve_wing_angle) before this can pass unconditionally."
    ),
    strict=True,
)
def test_keel_drag_power_is_never_positive() -> None:
    hydro = HydroDynamicsForceComputation()

    for nu in random_velocities(seed=5):
        force = hydro.keel_force(Vec4(nu), roll_rad=0.0)
        # Per the design intent in keel_force's docstring, the keel should oppose roll rate
        # and leeway (damping); power delivered to the boat must never be positive.
        power = force.data @ nu
        assert power <= 1e-9, f"keel injects energy: nu={nu} force={force.data}"


# --- The master energy test ------------------------------------------------------


def total_mechanical_energy(kinematics: BoatKinematics) -> float:
    """Kinetic energy with the total (rigid-body + added) mass, plus the roll potential
    energy stored against the hydrostatic restoring moment K = -rho*g*V*GZ(roll)."""
    nu = kinematics.nu.data
    kinetic = 0.5 * nu @ M_TOTAL @ nu
    potential = (
        WATER_DENSITY
        * EARTH_GRAVITY
        * DISPLACED_VOLUME
        * restoring_potential_energy(
            heel_angle_rad=kinematics.pose.p,
            gm=METACENTRIC_HEIGHT,
            bm=BM_METACENTRIC_RADIUS,
            deck_edge_immersion_rad=DECK_EDGE_IMMERSION_ANGLE_RAD,
        )
    )
    return float(kinetic + potential)


def test_coasting_boat_never_gains_energy() -> None:
    """With no wind, no current, and no actuation, a moving boat can only coast and slow
    down. If total mechanical energy increases on any timestep, some force term is
    injecting energy — the exact failure mode that makes the simulation diverge.
    """
    kinematics = BoatKinematics(timestep=0.01)
    kinematics.kinematics.nu = Vec4.from_xypr(1.0, 0.3, 0.2, 0.1)
    forces = TotalForceComputation()

    energy = total_mechanical_energy(kinematics)
    initial_energy = energy
    for step in range(2000):  # 20 seconds of sim time
        tau = forces.compute_total_force(
            kinematics,
            true_wind_speed_mps=0.0,
            true_wind_bearing_rad=0.0,
            ocean_current_speed_mps=0.0,
            ocean_current_bearing_rad=0.0,
            delta_r_rad=0.0,
            delta_tab_rad=0.0,
        )
        kinematics.step(kinematics.nu, tau)

        assert np.all(np.isfinite(kinematics.nu.data)), f"non-finite state at step {step}"
        next_energy = total_mechanical_energy(kinematics)
        assert next_energy <= energy * (1.0 + 1e-9) + 1e-9, (
            f"energy increased at step {step}: {energy:.6f} -> {next_energy:.6f} J "
            f"(nu={kinematics.nu.data})"
        )
        energy = next_energy

    # And it must actually dissipate, not just hold steady.
    assert energy < initial_energy
