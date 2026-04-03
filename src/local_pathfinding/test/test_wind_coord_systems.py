import math

import pytest

import local_pathfinding.wind_coord_systems as wcs


@pytest.mark.parametrize(
    "boat_heading_deg_gc, aw_direction_deg_bc, expected",
    [
        (0.0, 0.0, 0.0),
        (45.0, 0.0, 45.0),
        (170.0, 30.0, -160.0),
        (-45.0, -135.0, 180.0),
        (0.0, 90.0, 90.0),
        (90.0, 90.0, 180.0),
        (180.0, 0.0, 180.0),
        (179.0, 1.0, 180.0),
        (-170.0, -30.0, 160.0),
        (120.0, -150.0, -30.0),
        (-45.0, 135.0, 90.0),
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
        (0.0, 0.0, 0.0),
        (45.0, 0.0, -45.0),
        (170.0, 30.0, -140.0),
        (0.0, 180.0, 180.0),
        (90.0, 0.0, -90.0),
        (90.0, 180.0, 90.0),
        (180.0, 0.0, 180.0),
        (-170.0, 150.0, -40.0),
        (120.0, -150.0, 90.0),
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
    aw_dir_deg_bc, aw_speed_kmph_bc, boat_heading_deg_gc,
    boat_speed_kmph_gc, expected_tw_dir_rad, expected_tw_speed_kmph
    ''',
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
    aw_dir_deg_bc: float,
    aw_speed_kmph_bc: float,
    boat_heading_deg_gc: float,
    boat_speed_kmph_gc: float,
    expected_tw_dir_rad: float,
    expected_tw_speed_kmph: float,
):
    tw_dir_rad, tw_speed_kmph = wcs.get_true_wind(
        aw_dir_deg_bc, aw_speed_kmph_bc, boat_heading_deg_gc, boat_speed_kmph_gc
    )

    # Convert radians to degrees for easier comparison
    tw_dir_deg = math.degrees(tw_dir_rad)

    assert tw_dir_deg == pytest.approx(
        expected=expected_tw_dir_rad, abs=1e-1
    ) and tw_speed_kmph == pytest.approx(expected=expected_tw_speed_kmph, abs=1e-1)


@pytest.mark.parametrize(
    '''
    tw_dir_deg_gc, tw_speed_kmph_gc, boat_heading_deg_gc,
    boat_speed_kmph_gc, expected_aw_dir_rad_bc, expected_aw_speed_kmph_bc
    ''',
    [
        (0, 0, 0, 0, 0, 0),
        (45.0, 3.0, 225.0, 3.0, 45.0, 6.0),
        (45.0, 3.0, 45.0, 3.0, 0.0, 0.0),
        (180.0, 10.0, 0.0, 5.0, 180.0, 15.0),
        (90.0, 8.0, -90.0, 8.0, 90.0, 16.0),
    ],
)
def test_get_apparent_wind_direction(
    tw_dir_deg_gc: float,
    tw_speed_kmph_gc: float,
    boat_heading_deg_gc: float,
    boat_speed_kmph_gc: float,
    expected_aw_dir_rad_bc: float,
    expected_aw_speed_kmph_bc: float,
):
    aw_dir_rad, aw_speed_kmph = wcs.get_apparent_wind(
        tw_dir_deg_gc, tw_speed_kmph_gc, boat_heading_deg_gc, boat_speed_kmph_gc
    )

    # Convert radians to degrees for easier comparison
    aw_dir_deg = math.degrees(aw_dir_rad)

    assert aw_dir_deg == pytest.approx(
        expected=expected_aw_dir_rad_bc, abs=1e-2
    ) and aw_speed_kmph == pytest.approx(expected=expected_aw_speed_kmph_bc, abs=1e-2)


@pytest.mark.parametrize(
    '''
    tw_dir_deg_gc, tw_speed_kmph_gc, boat_heading_deg_gc,
    boat_speed_gc, expected_aw_direction_deg_bc, expected_aw_speed_kmph_bc
    ''',  # noqa
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
    tw_dir_deg_gc: float,
    tw_speed_kmph_gc: float,
    boat_heading_deg_gc: float,
    boat_speed_gc: float,
    expected_aw_direction_deg_bc: float,
    expected_aw_speed_kmph_bc: float,
):
    aw_dir_deg, aw_speed_kmph = wcs.get_apparent_wind(
        tw_dir_deg_gc, tw_speed_kmph_gc, boat_heading_deg_gc, boat_speed_gc, ret_rad=False
    )

    aw_dir_boat_coord_deg = wcs.global_to_boat_coordinate(boat_heading_deg_gc, aw_dir_deg)

    assert aw_dir_boat_coord_deg == pytest.approx(
        expected=expected_aw_direction_deg_bc, abs=1
    ), f"Apparent wind direction in boat frame mismatch: {aw_dir_boat_coord_deg} != {expected_aw_direction_deg_bc}"  # noqa
    assert aw_speed_kmph == pytest.approx(
        expected=expected_aw_speed_kmph_bc, abs=1
    ), f"Apparent wind speed mismatch: {aw_speed_kmph} != {expected_aw_speed_kmph_bc}"


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
