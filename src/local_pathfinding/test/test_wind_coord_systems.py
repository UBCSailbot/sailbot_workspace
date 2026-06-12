import math

import pytest

import local_pathfinding.wind_coord_systems as wcs


@pytest.mark.parametrize(
    "boat_heading_deg_gc, aw_direction_deg_bc, expected",
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
        (45.0, 90.0, -45.0)
    ],
)
def test_boat_to_global_coordinate(boat_heading_deg_gc: float,
                                   aw_direction_deg_bc: float,
                                   expected: float):
    assert wcs.boat_to_global_coordinate(boat_heading_deg_gc,
                                         aw_direction_deg_bc) == pytest.approx(
        expected
    ), "incorrect angle conversion"


@pytest.mark.parametrize(
    "boat_heading_deg_gc, tw_direction_deg_gc, expected",
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
    boat_heading_deg_gc: float, tw_direction_deg_gc: float, expected: float
):
    assert wcs.global_to_boat_coordinate(boat_heading_deg_gc,
                                         tw_direction_deg_gc) == pytest.approx(
        expected
    ), "incorrect angle conversion"  # noqa


@pytest.mark.parametrize(
    '''
    aw_dir_deg_bc, aw_speed_kmph, boat_heading_deg_gc,
    boat_speed_kmph, expected_tw_dir_deg, expected_tw_speed_kmph
    ''',
    [
        (0, 0, 0, 0, 0, 0),
        (10, 0, 10, 10, 10, 10),
        (179, 17, 179, 9, -3.13, 8.00),
        (180, 17, 179, 9, -1.00, 8.00),
        (140, 17, 45, 9, 155.22, 11.64),
        (80, 5, -70, 8, -40.94, 10.15),
        (70, 10.579, 180.0, 3.452, -125.36, 12.20),
        (70, 10.600, 180.0, 3.518, -125.66, 12.25),
        (70, 10.601, -79.198, 3.518, -24.83, 12.25),
        (-8, 13.044, -79.198, 3.085, -94.40, 16.10),
        (-8, 13.451, -52.021, 3.497, -58.39, 16.92),
        (-31, 11.693, -52.021, 2.060, -78.46, 13.50),
        (-37, 10.100, -71.758, 0.127, -108.20, 10.19),
        (-16, 10.745, -71.758, 0.782, -86.71, 11.50),
        (-15, 11.958, -71.765, 2.044, -84.59, 13.94),
        (-14, 12.370, -71.765, 2.470, -83.41, 13.94),
    ],
)
def test_get_true_wind_direction(
    aw_dir_deg_bc: float,
    aw_speed_kmph: float,
    boat_heading_deg_gc: float,
    boat_speed_kmph: float,
    expected_tw_dir_deg: float,
    expected_tw_speed_kmph: float,
):
    tw_dir_rad, tw_speed_kmph = wcs.get_true_wind(
        aw_dir_deg_bc, aw_speed_kmph, boat_heading_deg_gc, boat_speed_kmph
    )

    # Convert radians to degrees for easier comparison
    tw_dir_deg = math.degrees(tw_dir_rad)

    assert tw_dir_deg == pytest.approx(
        expected=expected_tw_dir_deg, abs=1e-1
    ) and tw_speed_kmph == pytest.approx(expected=expected_tw_speed_kmph, abs=1e-1)


@pytest.mark.parametrize(
    '''
    tw_dir_deg_gc, tw_speed_kmph, boat_heading_deg_gc,
    boat_speed_kmph, expected_aw_dir_deg_bc, expected_aw_speed_kmph
    ''',
    [
        (0, 0, 0, 0, 0, 0),
        (45.0, 3.0, 225.0, 3.0, 180.0, 6.0),
        (45.0, 3.0, 45.0, 3.0, -45.0, 0.0),
        (180.0, 10.0, 0.0, 5.0, 180.0, 15.0),
        (90.0, 8.0, -90.0, 8.0, 180.0, 16.0),
    ],
)
def test_get_apparent_wind_direction(
    tw_dir_deg_gc: float,
    tw_speed_kmph: float,
    boat_heading_deg_gc: float,
    boat_speed_kmph: float,
    expected_aw_dir_deg_bc: float,
    expected_aw_speed_kmph: float,
):
    aw_dir_rad_bc, aw_speed_kmph = wcs.get_apparent_wind(
        tw_dir_deg_gc, tw_speed_kmph, boat_heading_deg_gc, boat_speed_kmph
    )

    # Convert radians to degrees for easier comparison
    aw_dir_deg_bc = math.degrees(aw_dir_rad_bc)

    assert aw_dir_deg_bc == pytest.approx(
        expected=expected_aw_dir_deg_bc, abs=1e-2
    ) and aw_speed_kmph == pytest.approx(expected=expected_aw_speed_kmph, abs=1e-2)


@pytest.mark.parametrize(
    '''
    tw_dir_deg_gc, tw_speed_kmph, boat_heading_deg_gc,
    boat_speed, expected_aw_direction_deg_bc, expected_aw_speed_kmph
    ''',  # noqa
    [
        (90.0, 10.0, 0.0, 0.0, -90.0, 10.0),
        (180.0, 10.0, 0.0, 5.0, 0.0, 15.0),
        (45.0, 3.0, 225.0, 3.0, 0.0, 6.0),
        (90.0, 8.0, -90.0, 8.0, 0.0, 16.0),
        (90.0, 10.0, 0.0, 5.0, -63.43, 11.18),
        (90.0, 10.0, 0.0, 2.63, -75.26, 10.34),
    ],
)
def test_mock_wind_sensor_pipeline(
    tw_dir_deg_gc: float,
    tw_speed_kmph: float,
    boat_heading_deg_gc: float,
    boat_speed: float,
    expected_aw_direction_deg_bc: float,
    expected_aw_speed_kmph: float,
):
    aw_dir_deg, aw_speed_kmph = wcs.get_apparent_wind(
        tw_dir_deg_gc, tw_speed_kmph, boat_heading_deg_gc, boat_speed, ret_rad=False
    )

    aw_dir_boat_coord_deg = wcs.global_to_boat_coordinate(boat_heading_deg_gc, aw_dir_deg)

    assert aw_dir_boat_coord_deg == pytest.approx(
        expected=expected_aw_direction_deg_bc, abs=1
    ), f"Apparent wind direction in boat frame mismatch: {aw_dir_boat_coord_deg} != {expected_aw_direction_deg_bc}"  # noqa
    assert aw_speed_kmph == pytest.approx(
        expected=expected_aw_speed_kmph, abs=1
    ), f"Apparent wind speed mismatch: {aw_speed_kmph} != {expected_aw_speed_kmph}"


@pytest.mark.parametrize(
    "boat_heading_rad_gc, tw_dir_rad_gc, expected",
    [
        (0, 0, 0),
        (0, math.radians(45), math.radians(45)),
        (0, math.radians(90), math.radians(90)),
        (0, math.radians(135), math.radians(135)),
        (0, math.radians(180), math.radians(180)),
        (0, math.radians(-45), math.radians(-45)),
        (0, math.radians(-90), math.radians(-90)),
        (0, math.radians(-135), math.radians(-135)),
        (math.radians(45), 0, math.radians(-45)),
        (math.radians(90), 0, math.radians(-90)),
        (math.radians(135), 0, math.radians(-135)),
        (math.radians(180), 0, math.radians(180)),
        (math.radians(-45), 0, math.radians(45)),
        (math.radians(-90), 0, math.radians(90)),
        (math.radians(-135), 0, math.radians(135)),
        (math.radians(-179), 0, math.radians(179)),
    ],
)
def test_get_true_wind_angle(boat_heading_rad_gc: float, tw_dir_rad_gc: float, expected: float):
    twa = wcs.get_true_wind_angle(boat_heading_rad_gc, tw_dir_rad_gc)
    assert twa == pytest.approx(expected, abs=0.1)
