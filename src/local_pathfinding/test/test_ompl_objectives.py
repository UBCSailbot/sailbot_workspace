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
            cs.XY(0.5, math.sqrt(3) / 2),
            0.0,
            9.0,
            0.7,
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
        # Corners of the table
        (math.radians(0), math.radians(0), 0, 0),
        (math.radians(-90), math.radians(90), 37.0, 18.5),
        (math.radians(0), math.radians(180), 0, 0),
        (math.radians(0), math.radians(0), 37.0, 0),
        # Edges of table
        (math.radians(-48), math.radians(22), 0, 0),
        (math.radians(-22), math.radians(140), 0, 0),
        (math.radians(63), math.radians(63), 9.3, 0),
        (math.radians(-81), math.radians(-81), 32.3, 0),
        # Other edge cases
        (math.radians(60), math.radians(-120), 10.6, 3.7),
        (math.radians(170), math.radians(-155), 37, 6.8),
        (math.radians(-50), math.radians(-152.7), 27.8, 15.8),
        (math.radians(-170), math.radians(160), 14.4, 1.2),
        (math.radians(0), math.radians(45), 18.5, 3.7),
        # General cases
        (math.radians(-20), math.radians(40), 12.0, 2.9),
        (math.radians(12.9), math.radians(-1), 5.3, 0),
    ],
)
def test_get_sailbot_speed(
    heading: float, wind_direction: float, wind_speed: float, expected: float
):
    speed = objectives.TimeObjective.get_sailbot_speed(heading, wind_direction, wind_speed)
    assert speed == pytest.approx(expected, abs=0.1)
