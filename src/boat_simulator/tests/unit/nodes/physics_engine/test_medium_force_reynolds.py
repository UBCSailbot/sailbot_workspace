"""Tests for Reynolds-number-dependent coefficient lookup in MediumForceComputation."""

import math

import numpy as np
import pytest

from boat_simulator.common.types import CoeffGrid, CoeffTable
from boat_simulator.nodes.physics_engine.fluid_forces import MediumForceComputation


def _grid(low_value: float, high_value: float) -> CoeffGrid:
    """Two-Re grid: ``low_value`` at Re=100, ``high_value`` at Re=1000, sampled at 5 deg."""
    low = CoeffTable(np.array([[0.0, 0.0], [5.0, low_value], [90.0, 0.0]], dtype=np.float64))
    high = CoeffTable(np.array([[0.0, 0.0], [5.0, high_value], [90.0, 0.0]], dtype=np.float64))
    return CoeffGrid(np.array([100.0, 1000.0], dtype=np.float64), (low, high))


def _medium(lift: CoeffGrid, drag: CoeffGrid) -> MediumForceComputation:
    # chord = 1 and kinematic_viscosity = 1 make Reynolds number equal to the flow speed.
    return MediumForceComputation(
        lift_coefficients=lift,
        drag_coefficients=drag,
        areas=1.0,
        fluid_density=1.0,
        chord=1.0,
        kinematic_viscosity=1.0,
    )


class TestMediumForceReynolds:
    def test_lift_tracks_reynolds_via_flow_speed(self):
        # Lift rises with Reynolds; drag falls with Reynolds (typical foil behaviour).
        medium = _medium(_grid(1.0, 2.0), _grid(0.5, 0.1))
        cl_low, cd_low, _ = medium.interpolate(5.0, flow_speed=100.0)
        cl_high, cd_high, _ = medium.interpolate(5.0, flow_speed=1000.0)
        assert math.isclose(cl_low, 1.0) and math.isclose(cl_high, 2.0)
        assert cl_high > cl_low
        assert cd_high < cd_low

    def test_force_uses_reynolds_dependent_coefficient(self):
        medium = _medium(_grid(1.0, 2.0), _grid(0.1, 0.1))
        from boat_simulator.common.types import Vec2

        # Flow along +x with the medium oriented so the angle of attack is 5 deg.
        lift_low, _, _ = medium.compute(Vec2.from_xy(100.0 * math.cos(math.radians(5.0)),
                                                     100.0 * math.sin(math.radians(5.0))), 0.0)
        lift_high, _, _ = medium.compute(Vec2.from_xy(1000.0 * math.cos(math.radians(5.0)),
                                                      1000.0 * math.sin(math.radians(5.0))), 0.0)
        # Force ~ 0.5*rho*Cl*A*v^2. Speed x10 -> v^2 x100, and Cl doubles (Re x10), so lift x200.
        assert math.isclose(lift_high / lift_low, 200.0, rel_tol=1e-6)

    def test_beyond_max_angle_returns_zero(self):
        medium = _medium(_grid(1.0, 2.0), _grid(0.5, 0.5))
        cl, cd, _ = medium.interpolate(95.0, flow_speed=500.0)  # > 90 deg tabulated max
        assert cl == 0.0 and cd == 0.0

    def test_symmetric_sign_convention(self):
        medium = _medium(_grid(1.0, 2.0), _grid(0.5, 0.5))
        cl_pos, cd_pos, _ = medium.interpolate(5.0, flow_speed=1000.0)
        cl_neg, cd_neg, _ = medium.interpolate(-5.0, flow_speed=1000.0)
        assert math.isclose(cl_neg, -cl_pos)  # lift is odd in angle of attack
        assert math.isclose(cd_neg, cd_pos)  # drag is even in angle of attack

    def test_zero_velocity_returns_zero_force(self):
        medium = _medium(_grid(1.0, 2.0), _grid(0.5, 0.5))
        from boat_simulator.common.types import Vec2

        lift, drag, _ = medium.compute(Vec2.from_xy(0.0, 0.0), 30.0)
        assert lift == 0.0 and drag == 0.0


@pytest.mark.parametrize("reynolds", [50_000, 200_000, 1_000_000])
def test_real_keel_grid_lift_increases_with_reynolds_near_stall(reynolds):
    """Sanity check on the generated NACA 0012 keel grid: max lift grows with Reynolds."""
    from boat_simulator.common.constants import BOAT_PROPERTIES

    grid = BOAT_PROPERTIES.keel_lift_coeffs
    # Near 15 deg the low-Re polar is already stalled while the high-Re polar still climbs.
    low = grid.interpolate(15.0, 50_000)
    high = grid.interpolate(15.0, 1_000_000)
    assert high > low
