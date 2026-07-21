import math

import pytest

import local_pathfinding.coord_systems as cs
import local_pathfinding.ompl_objectives as ob


def test_soft_objective_weights_sum_to_one():
    assert (
        ob.WIND_OBJECTIVE_WEIGHT
        + ob.TIME_OBJECTIVE_WEIGHT
        + ob.DEVIATION_OBJECTIVE_WEIGHT
        + ob.SEGMENT_COUNT_OBJECTIVE_WEIGHT
    ) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "cs1, cs2, tw_dir_rad_gc, tw_speed_kmph, expected",
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
    cs1: tuple, cs2: tuple, tw_dir_rad_gc: float, tw_speed_kmph: float, expected: float
):
    s1 = cs.XY(*cs1)
    s2 = cs.XY(*cs2)
    assert ob.TimeObjective.time_cost(
        s1, s2, tw_dir_rad_gc, tw_speed_kmph
    ) == pytest.approx(expected, abs=0.1)


@pytest.mark.parametrize(
    "s1, s2, start, goal, expected",
    [
        # Segment midpoint sits on the rhumb line -> zero cost
        (cs.XY(2.0, 0.0), cs.XY(4.0, 0.0), cs.XY(0.0, 0.0), cs.XY(10.0, 0.0), 0.0),
        # Midpoint at half the max deviation -> 0.5**4 = 0.0625
        (cs.XY(2.0, 2.5), cs.XY(4.0, 2.5), cs.XY(0.0, 0.0), cs.XY(10.0, 0.0), 0.0625),
        # Midpoint exactly at the max deviation -> saturated cost
        (cs.XY(2.0, 5.0), cs.XY(4.0, 5.0), cs.XY(0.0, 0.0), cs.XY(10.0, 0.0), 1.0),
        # Midpoint beyond the max deviation -> clamped to 1
        (cs.XY(2.0, 6.0), cs.XY(4.0, 6.0), cs.XY(0.0, 0.0), cs.XY(10.0, 0.0), 1.0),
        # Degenerate start == goal -> no axis, zero cost
        (cs.XY(2.0, 2.0), cs.XY(3.0, 3.0), cs.XY(0.0, 0.0), cs.XY(0.0, 0.0), 0.0),
    ],
)
def test_deviation_cost(
    s1: cs.XY, s2: cs.XY, start: cs.XY, goal: cs.XY, expected: float
):
    assert ob.DeviationObjective.deviation_cost(s1, s2, start, goal) == pytest.approx(
        expected, abs=0.01
    )


@pytest.mark.parametrize(
    "heading, tw_dir_rad_gc, tw_speed_kmph, expected",
    [
        # 0 deg flow-relative angle (segment travels with the airflow)
        (math.radians(0), math.radians(0), 0, 0),
        (math.radians(0), math.radians(0), 75.0, 0),
        # 180 deg flow-relative angle (segment travels against the airflow)
        (math.radians(0), math.radians(180), 0, 0),
        (math.radians(0), math.radians(180), 75.0, 10),
    ],
)
def test_get_sailbot_speed(
    heading: float, tw_dir_rad_gc: float, tw_speed_kmph: float, expected: float
):
    speed = ob.TimeObjective.get_sailbot_speed(heading, tw_dir_rad_gc, tw_speed_kmph)
    assert speed == pytest.approx(expected, abs=0.1)
