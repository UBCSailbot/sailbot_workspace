import math

import pytest

import local_pathfinding.coord_systems as cs
import local_pathfinding.ompl_objectives as ob


@pytest.mark.parametrize(
    "cs1, cs2, tw_direction_rad_gc, tw_speed_kmph, expected",
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
def test_time_cost(
    cs1: tuple, cs2: tuple, tw_direction_rad_gc: float, tw_speed_kmph: float, expected: float
):
    s1 = cs.XY(*cs1)
    s2 = cs.XY(*cs2)
    assert ob.TimeObjective.time_cost(
        s1, s2, tw_direction_rad_gc, tw_speed_kmph
    ) == pytest.approx(expected, abs=0.1)


@pytest.mark.parametrize(
    "heading, tw_direction_rad_gc, tw_speed_kmph, expected",
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
    heading: float, tw_direction_rad_gc: float, tw_speed_kmph: float, expected: float
):
    speed = ob.TimeObjective.get_sailbot_speed(heading, tw_direction_rad_gc, tw_speed_kmph)
    assert speed == pytest.approx(expected, abs=0.1)
