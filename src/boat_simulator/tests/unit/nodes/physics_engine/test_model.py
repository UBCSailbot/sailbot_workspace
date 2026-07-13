"""Tests classes and functions in boat_simulator/nodes/physics_engine/model.py"""

import math

import numpy as np
import pytest

from boat_simulator.common.angle_conventions import Heading, RudderAngle, TrimTabAngle
from boat_simulator.common.constants import BOAT_PROPERTIES
from boat_simulator.common.conventions import NED, Velocity
from boat_simulator.common.types import Vec2, Vec4
from boat_simulator.nodes.physics_engine.model import BoatState
from custom_interfaces.msg import HelperLatLon
from local_pathfinding.coord_systems import XY, meters_to_km, xy_to_latlon

TIMESTEP = 0.1
REFERENCE_LATLON = HelperLatLon(latitude=49.2827, longitude=-123.1207)

ZERO_RUDDER = RudderAngle(0.0)
ZERO_TRIM_TAB = TrimTabAngle(0.0)


def make_boat_state(timestep: float = TIMESTEP) -> BoatState:
    return BoatState(timestep=timestep, reference_latlon=REFERENCE_LATLON)


def _set_pose(boat_state: BoatState, x=0.0, y=0.0, roll=0.0, yaw=0.0) -> None:
    """Directly sets the boat's pose for test setup.

    Driving the boat to a specific pose through `step()` would mean solving the force model
    backwards, so this reaches past the public API into the underlying `BoatKinematics` to
    set up non-trivial state for the property tests below.
    """
    boat_state._BoatState__kinematics_computation.kinematics.pose = (  # type: ignore[attr-defined]
        Vec4.from_xypr(x, y, roll, yaw)
    )


def _set_nu(boat_state: BoatState, u=0.0, v=0.0, p=0.0, r=0.0) -> None:
    boat_state._BoatState__kinematics_computation.kinematics.nu = (  # type: ignore[attr-defined]
        Vec4.from_xypr(u, v, p, r)
    )


# --- construction -------------------------------------------------------------


def test_construction_stores_timestep() -> None:
    boat_state = make_boat_state()

    assert boat_state.timestep == pytest.approx(TIMESTEP)


def test_construction_initializes_zero_state() -> None:
    boat_state = make_boat_state()

    np.testing.assert_array_equal(boat_state.pose.data, np.zeros(4))
    np.testing.assert_array_equal(boat_state.nu.data, np.zeros(4))
    np.testing.assert_array_equal(boat_state.nu_dot.data, np.zeros(4))


def test_construction_uses_boat_properties_mass_and_inertia() -> None:
    boat_state = make_boat_state()

    assert boat_state.boat_mass == pytest.approx(BOAT_PROPERTIES.mass)
    np.testing.assert_array_equal(boat_state.inertia.data, BOAT_PROPERTIES.inertia.data)
    np.testing.assert_allclose(
        boat_state.inertia_inverse.data, np.linalg.inv(BOAT_PROPERTIES.inertia.data)
    )


# --- linear_speed ---------------------------------------------------------------


def test_linear_speed_is_norm_of_surge_and_sway() -> None:
    boat_state = make_boat_state()
    _set_nu(boat_state, u=3.0, v=4.0)

    assert boat_state.linear_speed == pytest.approx(5.0)


def test_linear_speed_ignores_roll_and_yaw_rate() -> None:
    boat_state = make_boat_state()
    _set_nu(boat_state, u=3.0, v=4.0, p=100.0, r=-100.0)

    assert boat_state.linear_speed == pytest.approx(5.0)


# --- true_bearing ----------------------------------------------------------------


def test_true_bearing_reflects_pose_yaw() -> None:
    boat_state = make_boat_state()
    yaw = 0.5
    _set_pose(boat_state, yaw=yaw)

    assert boat_state.true_bearing == Heading(yaw)


def test_true_bearing_wraps_to_pi() -> None:
    boat_state = make_boat_state()
    _set_pose(boat_state, yaw=math.pi + 0.1)

    bearing = boat_state.true_bearing
    assert -math.pi <= bearing.radians < math.pi
    assert bearing == Heading(math.pi + 0.1)


# --- global_lat_lon_position -------------------------------------------------------


def test_global_lat_lon_position_at_origin_is_reference() -> None:
    boat_state = make_boat_state()

    lat, lon = boat_state.global_lat_lon_position
    assert lat == pytest.approx(REFERENCE_LATLON.latitude)
    assert lon == pytest.approx(REFERENCE_LATLON.longitude)


def test_global_lat_lon_position_matches_xy_to_latlon() -> None:
    boat_state = make_boat_state()
    north_m, east_m = 1500.0, -800.0
    _set_pose(boat_state, x=north_m, y=east_m)

    expected = xy_to_latlon(REFERENCE_LATLON, XY(x=meters_to_km(north_m), y=meters_to_km(east_m)))

    lat, lon = boat_state.global_lat_lon_position
    assert lat == pytest.approx(expected.latitude)
    assert lon == pytest.approx(expected.longitude)


# --- step() ------------------------------------------------------------------------
# `step()` assembles the net force via `TotalForceComputation` and integrates it via
# `BoatKinematics.step`. To test the integration wiring in isolation, the force computation is
# stubbed to zero; the force model itself is covered by the fluid_forces tests.


def _stub_zero_net_force(boat_state: BoatState) -> None:
    """Replaces the kinetics computation's `compute_total_force` with one returning zero, so
    `step()` exercises only the kinematics integration."""

    def zero_force(**_kwargs) -> Vec4:
        return Vec4.from_xypr(0.0, 0.0, 0.0, 0.0)

    boat_state._BoatState__kinetics_computation.compute_total_force = (  # type: ignore[attr-defined]  # noqa: E501
        zero_force
    )


def test_step_with_zero_net_force_leaves_boat_at_rest() -> None:
    boat_state = make_boat_state()
    _stub_zero_net_force(boat_state)
    wind = Vec2[Velocity, NED].from_xy(12.0, 4.0)
    water = Vec2[Velocity, NED].from_xy(-1.0, 2.0)
    rudder = RudderAngle(math.radians(10.0))
    trim_tab = TrimTabAngle(math.radians(-5.0))

    boat_state.step(wind, water, rudder, trim_tab)

    np.testing.assert_array_equal(boat_state.nu.data, np.zeros(4))
    np.testing.assert_array_equal(boat_state.nu_dot.data, np.zeros(4))
    np.testing.assert_array_equal(boat_state.pose.data, np.zeros(4))


def test_step_with_wind_and_current_moves_boat_from_rest() -> None:
    # With the real force model wired in, wind and ocean current must produce a nonzero net
    # force on a boat at rest.
    boat_state = make_boat_state()
    wind = Vec2[Velocity, NED].from_xy(12.0, 4.0)
    water = Vec2[Velocity, NED].from_xy(-1.0, 2.0)

    boat_state.step(wind, water, ZERO_RUDDER, ZERO_TRIM_TAB)

    assert not np.allclose(boat_state.nu.data, np.zeros(4))


def test_step_does_not_raise_with_nonzero_boat_velocity() -> None:
    # Exercises the apparent wind/current computation, which depends on the boat's own nu.
    boat_state = make_boat_state()
    _set_nu(boat_state, u=2.0, v=-1.0)
    wind = Vec2[Velocity, NED].from_xy(6.0, 0.0)
    water = Vec2[Velocity, NED].from_xy(0.0, 3.0)

    boat_state.step(wind, water, ZERO_RUDDER, ZERO_TRIM_TAB)
