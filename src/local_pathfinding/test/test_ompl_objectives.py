import math

import pytest

import local_pathfinding.coord_systems as cs
import local_pathfinding.ompl_objectives as objectives


@pytest.mark.parametrize(
    "cs1,cs2,wind_direction_rad,wind_speed_kmph,expected",
    [
        (
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            0.0,
            0.0,
            1.0,
        ),  # ZERO SPEED MAX COST
        (
            cs.XY(0, 0),
            cs.XY(1.0, 1.0),
            0.0,
            9.0,
            0.2,
        ),  # SLOW & HIGH COST
        (cs.XY(0, 0), cs.XY(1, -1), 0.0, 37.0, 0.0),  # FAST & LOW COST
    ],
)
def test_speed_cost(
    cs1: tuple, cs2: tuple, wind_direction_rad: float, wind_speed_kmph: float, expected: float
):
    s1 = cs.XY(*cs1)
    s2 = cs.XY(*cs2)
    assert objectives.TimeObjective.time_cost(
        s1, s2, wind_direction_rad, wind_speed_kmph
    ) == pytest.approx(expected, abs=0.1)


@pytest.mark.parametrize(
    "heading,wind_direction,wind_speed,expected",
    [
        # 0 deg true wind angle (irons)
        (math.radians(0), math.radians(0), 0, 0),
        (math.radians(0), math.radians(0), 75.0, 0),
        # 180 deg true wind angle (dead run)
        (math.radians(0), math.radians(180), 0, 0),
        (math.radians(0), math.radians(180), 75.0, 10),
    ],
)
def test_get_sailbot_speed(
    heading: float, wind_direction: float, wind_speed: float, expected: float
):
    speed = objectives.TimeObjective.get_sailbot_speed(heading, wind_direction, wind_speed)
    assert speed == pytest.approx(expected, abs=0.1)
