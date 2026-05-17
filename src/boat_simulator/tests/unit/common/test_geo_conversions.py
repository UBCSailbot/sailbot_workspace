"""Tests for boat_simulator/common/geo_conversions.py."""

import math

import numpy as np
import pytest
from pyproj import Geod

import boat_simulator.common.constants as Constants
from boat_simulator.common.geo_conversions import local_position_to_gps_lat_lon


GEODESIC = Geod(ellps="WGS84")


def heading_difference_deg(heading_a: float, heading_b: float) -> float:
    return abs((heading_a - heading_b + 180.0) % 360.0 - 180.0)


def test_local_position_origin_to_gps_origin():
    latitude, longitude = local_position_to_gps_lat_lon(np.zeros(3))

    assert latitude == pytest.approx(Constants.SIM_GPS_ORIGIN_LATITUDE)
    assert longitude == pytest.approx(Constants.SIM_GPS_ORIGIN_LONGITUDE)


@pytest.mark.parametrize(
    "local_position_m",
    [
        np.array([1000.0, 0.0, 0.0]),
        np.array([0.0, 1000.0]),
        np.array([-300.0, -400.0, 25.0]),
    ],
)
def test_local_position_to_gps_lat_lon_preserves_distance_and_bearing(local_position_m):
    latitude, longitude = local_position_to_gps_lat_lon(local_position_m)

    bearing_deg, _, distance_m = GEODESIC.inv(
        Constants.SIM_GPS_ORIGIN_LONGITUDE,
        Constants.SIM_GPS_ORIGIN_LATITUDE,
        longitude,
        latitude,
    )
    expected_bearing_deg = math.degrees(math.atan2(local_position_m[0], local_position_m[1]))
    expected_distance_m = math.hypot(local_position_m[0], local_position_m[1])

    assert distance_m == pytest.approx(expected_distance_m)
    assert heading_difference_deg(bearing_deg, expected_bearing_deg) == pytest.approx(0.0, abs=1e-6)  # noqa


def test_local_position_to_gps_lat_lon_ignores_vertical_offset():
    surface_position = local_position_to_gps_lat_lon([20.0, 30.0, 0.0])
    elevated_position = local_position_to_gps_lat_lon([20.0, 30.0, 100.0])

    assert elevated_position == pytest.approx(surface_position)


def test_local_position_to_gps_lat_lon_handles_antimeridian():
    _, longitude = local_position_to_gps_lat_lon(
        [1000.0, 0.0],
        origin_latitude=0.0,
        origin_longitude=179.999,
    )

    assert -180.0 <= longitude <= 180.0


def test_local_position_to_gps_lat_lon_handles_high_latitude_origin():
    latitude, longitude = local_position_to_gps_lat_lon(
        [1000.0, 0.0],
        origin_latitude=89.9,
        origin_longitude=0.0,
    )

    assert math.isfinite(latitude)
    assert math.isfinite(longitude)
    assert -90.0 <= latitude <= 90.0
    assert -180.0 <= longitude <= 180.0


@pytest.mark.parametrize(
    "local_position_m",
    [
        [0.0],
        [[0.0, 0.0]],
        [float("nan"), 0.0],
        [0.0, float("inf")],
        ["east", 0.0],
    ],
)
def test_local_position_to_gps_lat_lon_rejects_invalid_local_position(local_position_m):
    with pytest.raises(ValueError):
        local_position_to_gps_lat_lon(local_position_m)


@pytest.mark.parametrize(
    "origin_latitude, origin_longitude",
    [
        (float("nan"), 0.0),
        (0.0, float("inf")),
        (-90.1, 0.0),
        (90.1, 0.0),
        (0.0, -180.1),
        (0.0, 180.1),
    ],
)
def test_local_position_to_gps_lat_lon_rejects_invalid_origin(
    origin_latitude, origin_longitude
):
    with pytest.raises(ValueError):
        local_position_to_gps_lat_lon(
            [0.0, 0.0],
            origin_latitude=origin_latitude,
            origin_longitude=origin_longitude,
        )
