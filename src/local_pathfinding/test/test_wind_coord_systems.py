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
    ]
)
def test_boat_to_global_coordinate(boat_heading: float, wind_direction: float, expected: float):
    assert (
        wcs.boat_to_global_coordinate(boat_heading, wind_direction) == pytest.approx(expected)
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
    ]
)
def test_global_to_boat_coordinate(boat_heading: float,
                                   global_wind_direction: float,
                                   expected: float):
    assert (
        wcs.global_to_boat_coordinate(boat_heading, global_wind_direction) == pytest.approx(expected) # noqa
    ), "incorrect angle conversion"


@pytest.mark.parametrize(
    "wind_direction_degrees,wind_speed,heading_degrees,speed,expected_direction, expected_speed",
    [
        (0, 0, 0, 0, 0, 0),
        (10, 0, 10, 10, 10, 10),
        (179, 17, 179, 9, 179, 26),
        (180, 17, 179, 9, 179.65, 26),
        (140, 17, 45, 9, 111.06, 18.52),
        (80, 5, -70, 8, -35.74, 4.44),
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
        expected=expected_direction, abs=1e-2
    ) and tw_speed_kmph == pytest.approx(expected=expected_speed, abs=1e-2)


@pytest.mark.parametrize(
    "tw_direction_degrees, tw_speed,heading_degrees,speed,expected_direction, expected_speed",
    [
        (0, 0, 0, 0, 0, 0),
        (45.0, 3.0, 225.0, 3.0, 45.0, 6.0),
        (45.0, 3.0, 45.0, 3.0, 0.0, 0.0),
        (180.0, 10.0, 0.0, 5.0, 180.0, 15.0),
        (90.0, 8.0, -90.0, 8.0, 90.0, 16.0),
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
        tw_direction_degrees,
        tw_speed, heading_degrees, speed
    )

    # Convert radians to degrees for easier comparison
    aw_dir_deg = math.degrees(aw_dir_rad)

    assert aw_dir_deg == pytest.approx(
        expected=expected_direction, abs=1e-2
    ) and aw_speed_kmph == pytest.approx(expected=expected_speed, abs=1e-2)
