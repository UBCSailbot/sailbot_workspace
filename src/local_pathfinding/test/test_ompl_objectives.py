import math

import pytest

import local_pathfinding.coord_systems as cs
import local_pathfinding.ompl_objectives as objectives
from local_pathfinding.ompl_objectives import (
    DOWNWIND_COST_MULTIPLIER,
    UPWIND_COST_MULTIPLIER,
)


@pytest.mark.parametrize(
    "cs1,cs2,wind_direction_rad,expected",
    [
        # Moving directly into wind (upwind)
        ((0, 0), (0, 1), 0.0, UPWIND_COST_MULTIPLIER * 1.0),
        # Moving perpendicular to wind (crosswind)
        ((0, 0), (1, 0), 0.0, 0.0),
        # Moving directly with the wind (downwind)
        ((0, 0), (0, -1), 0.0, DOWNWIND_COST_MULTIPLIER * 1.0),
        # Moving at 45° off upwind
        ((0, 0), (1, 1), 0.0, UPWIND_COST_MULTIPLIER * math.cos(math.radians(45))),
        # Moving 30° off upwind
        (
            (0, 0),
            (math.sin(math.radians(30)), math.cos(math.radians(30))),
            0.0,
            UPWIND_COST_MULTIPLIER * math.cos(math.radians(30)),
        ),
        # Moving 135° off upwind (45° off downwind)
        (
            (0, 0),
            (math.sin(math.radians(135)), math.cos(math.radians(135))),
            0.0,
            DOWNWIND_COST_MULTIPLIER * abs(math.cos(math.radians(135))),
        ),
        # Wind from 179°, boat moving North
        (
            (0, 0),
            (0, 1),
            math.radians(179.0),
            DOWNWIND_COST_MULTIPLIER * math.cos(math.radians(1)),
        ),
        # Wind from -179°, boat moving North
        (
            (0, 0),
            (0, 1),
            math.radians(-179.0),
            DOWNWIND_COST_MULTIPLIER * math.cos(math.radians(1)),
        ),
    ],
)
def test_wind_direction_cost(cs1: tuple, cs2: tuple, wind_direction_rad: float, expected: float):
    s1 = cs.XY(*cs1)
    s2 = cs.XY(*cs2)
    assert objectives.WindObjective.wind_direction_cost(
        s1, s2, wind_direction_rad
    ) == pytest.approx(expected, abs=1e-3)


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
