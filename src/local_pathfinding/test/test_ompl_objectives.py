import math

import pytest
from ompl import base as ompl_base
from ompl import geometric as ompl_geometric

import local_pathfinding.coord_systems as cs
import local_pathfinding.ompl_objectives as ob
from local_pathfinding.ompl_objectives import (NO_GO_ZONE, WIND_COST_SIN_EXPONENT)


def make_simple_setup():
    space = ompl_base.SE2StateSpace()
    bounds = ompl_base.RealVectorBounds(2)
    bounds.setLow(0, -10.0)
    bounds.setLow(1, -10.0)
    bounds.setHigh(0, 10.0)
    bounds.setHigh(1, 10.0)
    space.setBounds(bounds)

    simple_setup = ompl_geometric.SimpleSetup(space)
    return simple_setup, space


def make_state(space, x: float, y: float):
    state = ompl_base.State(space)
    state().setXY(x, y)
    return state


def get_goal_direction_cost(
    s1: cs.XY,
    s2: cs.XY,
    goal_position_in_xy: cs.XY = cs.XY(1.0, 0.0),
) -> float:
    simple_setup, space = make_simple_setup()
    objective = ob.GoalDirectionObjective(
        simple_setup.getSpaceInformation(),
        goal_position_in_xy,
    )

    state1 = make_state(space, s1.x, s1.y)
    state2 = make_state(space, s2.x, s2.y)
    return objective.motionCost(state1(), state2()).value()


@pytest.mark.parametrize(
    "angle_deg, expected_cost",
    [
        (
            # Normal movement straight toward the goal
            0.0,
            0.0,
        ),
        (
            # Movement exactly 90 degrees to the goal direction should be allowed
            90.0,
            0.0,
        ),
        (
            # Movement 91 degrees to the goal direction is slightly backward
            91.0,
            float("inf"),
        ),
        (
            # Movement 81 degrees to the goal direction still makes forward progress
            81.0,
            0.0,
        ),
        (
            # Large wrong-direction angle, clearly not moving toward the goal
            135.0,
            float("inf"),
        ),
        (
            # Large wrong-direction angle, directly away from the goal
            180.0,
            float("inf"),
        ),
    ],
)
def test_goal_direction_motion_cost_for_segment_angles(angle_deg: float, expected_cost: float):
    s1 = cs.XY(0.0, 0.0)
    angle_rad = math.radians(angle_deg)
    s2 = cs.XY(math.cos(angle_rad), math.sin(angle_rad))

    assert get_goal_direction_cost(s1, s2) == expected_cost


def test_goal_direction_motion_cost_allows_tolerance():
    # Movement that is barely backward due to numerical jitter should be allowed
    s1 = cs.XY(0.0, 0.0)
    s2 = cs.XY(-ob.GOAL_DIRECTION_TOLERANCE / 2, 1.0)

    assert get_goal_direction_cost(s1, s2) == 0.0


def test_goal_direction_motion_cost_uses_current_segment_start_to_goal():
    # Movement from an offset segment start toward the goal should be allowed
    s1 = cs.XY(0.0, 1.0)
    s2 = cs.XY(0.5, 0.5)

    assert get_goal_direction_cost(s1, s2) == 0.0


def test_goal_direction_motion_cost_rejects_movement_away_from_current_goal_direction():
    # Movement from an offset segment start away from the goal should be rejected
    s1 = cs.XY(0.0, 1.0)
    s2 = cs.XY(-0.5, 1.5)

    assert get_goal_direction_cost(s1, s2) == float("inf")


@pytest.mark.parametrize(
    "s1, s2, tw_direction_rad_gc, expected",
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
            math.sin(2*(NO_GO_ZONE - 0.0001)) ** WIND_COST_SIN_EXPONENT
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
            math.sin(2*math.pi / 3) ** WIND_COST_SIN_EXPONENT
        ),
        (
            # Boat heading north, wind heading at -55 degrees (-0.959931 radians)
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            -0.959931,
            math.sin(2*(-0.959931)) ** WIND_COST_SIN_EXPONENT
        ),
        (
            # Boat heading north, wind heading at 50 degrees (0.872665 radians)
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            0.872665,
            math.sin(2*0.872665) ** WIND_COST_SIN_EXPONENT
        )
    ],
)
def test_wind_direction_cost(
    s1: cs.XY, s2: cs.XY, tw_direction_rad_gc: float, expected: float
):
    result = ob.WindObjective.wind_direction_cost(s1, s2, tw_direction_rad_gc)
    assert result == pytest.approx(expected, abs=1e-12)


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
