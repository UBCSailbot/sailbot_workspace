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
        (0.0, 0.0, -135.0)
    ]
)
def test_boat_to_global_coordinate(boat_heading: float, wind_direction: float, expected: float):
    assert (
        wcs.boat_to_global_coordinate(boat_heading, wind_direction) == pytest.approx(expected)
    ), "incorrect angle conversion"
