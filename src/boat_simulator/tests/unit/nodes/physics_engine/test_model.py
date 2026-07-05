"""Tests classes and functions in boat_simulator/nodes/physics_engine/model.py"""

import math

import numpy as np
import pytest

from boat_simulator.common.angle_conventions import Heading, RudderAngle, TrimTabAngle
from boat_simulator.common.constants import BOAT_PROPERTIES
from boat_simulator.common.conventions import NED, Velocity
from boat_simulator.common.types import Vec2, Vec4
from boat_simulator.common.utils import ned_to_body_rotation_matrix
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

    `__compute_net_force_and_torque` is currently stubbed to always return zero (see the TODO
    in model.py), so `step()` alone cannot drive the boat away from rest. This reaches past the
    public API into the underlying `BoatKinematics` to set up non-trivial state for the
    property/rotation tests below.
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


# --- __ned_to_body_velocity (rotation used by step()) -----------------------------
# `__compute_net_force_and_torque` currently discards the rotated wind/water velocities (it's
# stubbed to zero, see model.py), so this rotation isn't observable through `step()` yet. These
# tests call the name-mangled private method directly to lock in its behavior so it isn't
# silently broken before the force model lands.


def test_ned_to_body_velocity_identity_at_zero_orientation() -> None:
    boat_state = make_boat_state()
    ned_vel = Vec2[Velocity, NED].from_xy(3.0, -2.0)

    body_vel = boat_state._BoatState__ned_to_body_velocity(ned_vel)  # type: ignore[attr-defined]

    assert body_vel.x == pytest.approx(3.0)
    assert body_vel.y == pytest.approx(-2.0)


def test_ned_to_body_velocity_rotates_by_roll_and_yaw() -> None:
    boat_state = make_boat_state()
    roll, yaw = math.pi / 6, math.pi / 3
    _set_pose(boat_state, roll=roll, yaw=yaw)
    ned_vel = Vec2[Velocity, NED].from_xy(5.0, 1.5)

    body_vel = boat_state._BoatState__ned_to_body_velocity(ned_vel)  # type: ignore[attr-defined]

    rotation = ned_to_body_rotation_matrix(roll_rad=roll, pitch_rad=0.0, yaw_rad=yaw)
    expected = rotation @ np.array([5.0, 1.5, 0.0])
    assert body_vel.x == pytest.approx(expected[0])
    assert body_vel.y == pytest.approx(expected[1])


# --- step() ------------------------------------------------------------------------
# The net force/torque calculation is currently stubbed to always return zero (see the TODO on
# `__compute_net_force_and_torque` in model.py), so `step()` cannot yet move the boat away from
# rest regardless of wind/water/rudder/trim-tab input. These tests lock in that (temporary)
# behavior so a silent regression is visible, and should be revisited once the force model is
# implemented.


def test_step_leaves_boat_at_rest_while_force_model_is_stubbed() -> None:
    boat_state = make_boat_state()
    wind = Vec2[Velocity, NED].from_xy(12.0, 4.0)
    water = Vec2[Velocity, NED].from_xy(-1.0, 2.0)
    rudder = RudderAngle(math.radians(10.0))
    trim_tab = TrimTabAngle(math.radians(-5.0))

    boat_state.step(wind, water, rudder, trim_tab)

    np.testing.assert_array_equal(boat_state.nu.data, np.zeros(4))
    np.testing.assert_array_equal(boat_state.nu_dot.data, np.zeros(4))
    np.testing.assert_array_equal(boat_state.pose.data, np.zeros(4))


def test_step_does_not_raise_with_nonzero_boat_velocity() -> None:
    # Exercises the rel_wind_vel/rel_water_vel computation (which depends on the boat's own
    # nu) even though the result is currently discarded by the force-model stub.
    boat_state = make_boat_state()
    _set_nu(boat_state, u=2.0, v=-1.0)
    wind = Vec2[Velocity, NED].from_xy(6.0, 0.0)
    water = Vec2[Velocity, NED].from_xy(0.0, 3.0)

    boat_state.step(wind, water, ZERO_RUDDER, ZERO_TRIM_TAB)
