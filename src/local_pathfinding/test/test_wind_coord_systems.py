import math

import pytest

import local_pathfinding.wind_coord_systems as wcs


@pytest.mark.parametrize(
    "boat_heading, wind_direction, expected",
    [
        (0.0, 0.0, 180.0),
        (45.0, 0.0, -135.0),
        (170.0, 30.0, 20.0),
        (-45.0, -135.0, 0.0),
        (0.0, 90.0, -90.0),
        (90.0, 90.0, 0.0),
        (180.0, 0.0, 0.0),
        (179.0, 1.0, 0.0),
        (-170.0, -30.0, -20.0),
        (120.0, -150.0, 150.0),
        (-45.0, 135.0, -90.0),
    ],
)
def test_boat_to_global_coordinate(boat_heading: float, wind_direction: float, expected: float):
    assert wcs.boat_to_global_coordinate(boat_heading, wind_direction) == pytest.approx(
        expected
    ), "incorrect angle conversion"


@pytest.mark.parametrize(
    "boat_heading, global_wind_direction, expected",
    [
        (0.0, 0.0, 180.0),
        (45.0, 0.0, 135.0),
        (170.0, 30.0, 40.0),
        (0.0, 180.0, 0.0),
        (90.0, 0.0, 90.0),
        (90.0, 180.0, -90.0),
        (180.0, 0.0, 0.0),
        (-170.0, 150.0, 140.0),
        (120.0, -150.0, -90.0),
    ],
)
def test_global_to_boat_coordinate(
    boat_heading: float, global_wind_direction: float, expected: float
):
    assert wcs.global_to_boat_coordinate(boat_heading, global_wind_direction) == pytest.approx(
        expected
    ), "incorrect angle conversion"  # noqa


@pytest.mark.parametrize(
    "wind_direction_degrees,wind_speed,heading_degrees,speed,expected_direction, expected_speed",
    [
        (0, 0, 0, 0, 0, 0),
        (10, 0, 10, 10, 10, 10),
        (179, 17, 179, 9, 179, 26),
        (180, 17, 179, 9, 179.65, 26),
        (140, 17, 45, 9, 111.06, 18.52),
        (80, 5, -70, 8, -35.74, 4.44),
        (70, 10.579, 180.0, 3.452, 89.04, 9.94),
        (70, 10.600, 180.0, 3.518, 89.38, 9.96),
        (70, 10.601, -79.198, 3.518, 56.64, 7.79),
        (-8, 13.044, -79.198, 3.085, -19.75, 14.34),
        (-8, 13.451, -52.021, 3.497, -16.65, 16.15),
        (-31, 11.693, -52.021, 2.060, -34.10, 13.64),
        (-37, 10.100, -71.758, 0.127, -37.40, 10.20),
        (-16, 10.745, -71.758, 0.782, -19.31, 11.20),
        (-15, 11.958, -71.765, 2.044, -22.45, 13.19),
        (-14, 12.370, -71.765, 2.470, -22.67, 13.85),
    ],
)
def test_get_true_wind_direction(
    wind_direction_degrees: float,
    wind_speed: float,
    heading_degrees: float,
    speed: float,
    expected_direction: float,
    expected_speed: float,
):
    tw_dir_rad, tw_speed_kmph = wcs.get_true_wind(
        wind_direction_degrees, wind_speed, heading_degrees, speed
    )

    # Convert radians to degrees for easier comparison
    tw_dir_deg = math.degrees(tw_dir_rad)

    assert tw_dir_deg == pytest.approx(
        expected=expected_direction, abs=1e-1
    ) and tw_speed_kmph == pytest.approx(expected=expected_speed, abs=1e-1)


@pytest.mark.parametrize(
    "tw_direction_degrees, tw_speed,heading_degrees,speed,expected_direction, expected_speed",
    [
        (0, 0, 0, 0, 0, 0),
        (45.0, 3.0, 225.0, 3.0, 45.0, 6.0),
        (45.0, 3.0, 45.0, 3.0, 0.0, 0.0),
        (180.0, 10.0, 0.0, 5.0, 180.0, 15.0),
        (90.0, 8.0, -90.0, 8.0, 90.0, 16.0),
        (-90.0, 30.0, -74.0, 0.2, -90.1, 29.81),
        (0.0, 30.0, 0.0, 0.0, 0.0, 30.0),
    ],
)
def test_get_apparent_wind_direction(
    tw_direction_degrees: float,
    tw_speed: float,
    heading_degrees: float,
    speed: float,
    expected_direction: float,
    expected_speed: float,
):
    aw_dir_rad, aw_speed_kmph = wcs.get_apparent_wind(
        tw_direction_degrees, tw_speed, heading_degrees, speed
    )

    # Convert radians to degrees for easier comparison
    aw_dir_deg = math.degrees(aw_dir_rad)

    assert aw_dir_deg == pytest.approx(
        expected=expected_direction, abs=1e-2
    ) and aw_speed_kmph == pytest.approx(expected=expected_speed, abs=1e-2)


@pytest.mark.parametrize(
    "tw_dir_deg, tw_speed_kmph, boat_heading_deg, boat_speed, expected_aw_boat_dir_deg, expected_aw_speed",  # noqa
    [
        (90.0, 10.0, 0.0, 0.0, -90.0, 10.0),
        (180.0, 10.0, 0.0, 5.0, 0.0, 15.0),
        (45.0, 3.0, 225.0, 3.0, 0.0, 6.0),
        (90.0, 8.0, -90.0, 8.0, 0.0, 16.0),
        (90.0, 10.0, 0.0, 5.0, -63, 11.18),
        (90.0, 10.0, 0.0, 2.63, -75.0, 10.34),
    ],
)
def test_mock_wind_sensor_pipeline(
    tw_dir_deg: float,
    tw_speed_kmph: float,
    boat_heading_deg: float,
    boat_speed: float,
    expected_aw_boat_dir_deg: float,
    expected_aw_speed: float,
):
    aw_dir_deg, aw_speed_kmph = wcs.get_apparent_wind(
        tw_dir_deg, tw_speed_kmph, boat_heading_deg, boat_speed, ret_rad=False
    )

    aw_dir_boat_coord_deg = wcs.global_to_boat_coordinate(boat_heading_deg, aw_dir_deg)

    assert aw_dir_boat_coord_deg == pytest.approx(
        expected=expected_aw_boat_dir_deg, abs=1
    ), f"Apparent wind direction in boat frame mismatch: {aw_dir_boat_coord_deg} != {expected_aw_boat_dir_deg}"  # noqa
    assert aw_speed_kmph == pytest.approx(
        expected=expected_aw_speed, abs=1
    ), f"Apparent wind speed mismatch: {aw_speed_kmph} != {expected_aw_speed}"
