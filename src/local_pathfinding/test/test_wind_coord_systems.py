import math
from typing import List

import custom_interfaces.msg as ci
import pytest
from shapely.geometry import MultiPolygon, Point, Polygon, box

import local_pathfinding.coord_systems as cs
import local_pathfinding.wind_coord_systems as wcs


@pytest.mark.parametrize(
    "boat_heading, wind_direction, expected",
    [
        (0.0, 0.0, -180.0),
        (45.0, 0.0, -135.0),
        (170.0, 30.0, 20.0),
        #TODO: Add more test cases before PR
    ]
)
def test_boat_to_global_coordinate(boat_heading: float, wind_direction: float, expected: float):
    assert (
        wcs.boat_to_global_coordinate(boat_heading, wind_direction) == pytest.approx(expected)
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
    true_wind_direction, true_wind_speed = wcs.get_true_wind(
        wind_direction_degrees, wind_speed, heading_degrees, speed
    )

    # Convert radians to degrees for easier comparison
    true_wind_direction_degrees = math.degrees(true_wind_direction)

    assert true_wind_direction_degrees == pytest.approx(
        expected=expected_direction, abs=1e-2
    ) and true_wind_speed == pytest.approx(expected=expected_speed, abs=1e-2)
