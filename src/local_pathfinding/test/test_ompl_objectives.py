import math

import pytest

import local_pathfinding.coord_systems as cs
import local_pathfinding.ompl_objectives as ob
from local_pathfinding.ompl_objectives import NO_GO_ZONE


@pytest.mark.parametrize(
    "s1, s2, tw_direction_rad, expected",
    [
        (
            # Wind direction and boat heading north
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            0.0,
            1.0
        ),
        (
            # Wind direction south and boat heading north
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            math.pi,
            1.0
        ),
        (
            # Boat heading north, wind at no-go zone at 45 degrees
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            NO_GO_ZONE,
            1.0
        ),
        (
            # Boat heading north, wind at no-go zone at -45 degrees
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            -NO_GO_ZONE,
            1.0
        ),
        (
            # Boat heading south, wind at no-go zone at 135 degrees
            cs.XY(0.0, 0.0),
            cs.XY(0.0, -1.0),
            math.pi - NO_GO_ZONE,
            1.0
        ),
        (
            # Boat heading south, wind at no-go zone at -135 degrees
            cs.XY(0.0, 0.0),
            cs.XY(0.0, -1.0),
            - math.pi + NO_GO_ZONE,
            1.0
        ),
        (
            # Boat heading east, wind just better than no-go zone
            cs.XY(0.0, 0.0),
            cs.XY(1.0, 0.0),
            NO_GO_ZONE - 0.0001,
            0.999998400001
        ),
        (
            # Boat heading west, wind heading north
            cs.XY(0.0, 0.0),
            cs.XY(-1.0, 0.0),
            0.0,
            0.0
        ),
        (
            # Boat heading west, wind heading south
            cs.XY(0.0, 0.0),
            cs.XY(-1.0, 0.0),
            math.pi,
            0.0
        ),
        (
            # Boat heading north, wind heading at 60 degrees
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            math.pi / 3,
            0.0000100565851616
        ),
        (
            # Boat heading north, wind heading at -55 degrees (-0.959931 radians)
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            0.959931,
            0.00690029338186
        ),
        (
            # Boat heading north, wind heading at 50 degrees (0.872665 radians)
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            0.872665,
            0.293840825304
        )
    ],
)
def test_wind_direction_cost(
    s1: cs.XY, s2: cs.XY, tw_direction_rad: float, expected: float
):
    result = ob.WindObjective.wind_direction_cost(s1, s2, tw_direction_rad)
    assert result == pytest.approx(expected, abs=1e-12)


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
    assert ob.TimeObjective.time_cost(
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
    speed = ob.TimeObjective.get_sailbot_speed(heading, wind_direction, wind_speed)
    assert speed == pytest.approx(expected, abs=0.1)
