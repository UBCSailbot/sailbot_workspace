import math

import pytest

import local_pathfinding.coord_systems as cs
import local_pathfinding.ompl_validity as ov


class MockState:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def getX(self) -> float:
        return self.x

    def getY(self) -> float:
        return self.y


def motion_makes_goal_progress(
    s1: cs.XY,
    s2: cs.XY,
    goal_position_in_xy: cs.XY = cs.XY(1.0, 0.0),
) -> bool:
    goal_progress_motion = ov.GoalProgressMotion(goal_position_in_xy)
    return goal_progress_motion._motion_makes_goal_progress(
        MockState(s1.x, s1.y),
        MockState(s2.x, s2.y),
    )


@pytest.mark.parametrize(
    "angle_deg, expected",
    [
        (
            # Normal movement straight toward the goal
            0.0,
            True,
        ),
        (
            # Movement exactly 90 degrees to the goal direction should be allowed
            90.0,
            True,
        ),
        (
            # Movement 91 degrees to the goal direction is slightly backward
            91.0,
            False,
        ),
        (
            # Movement 81 degrees to the goal direction still makes forward progress
            81.0,
            True,
        ),
        (
            # Large wrong-direction angle, clearly not moving toward the goal
            135.0,
            False,
        ),
        (
            # Large wrong-direction angle, directly away from the goal
            180.0,
            False,
        ),
    ],
)
def test_goal_progress_motion_for_segment_angles(angle_deg: float, expected: bool):
    s1 = cs.XY(0.0, 0.0)
    angle_rad = math.radians(angle_deg)
    s2 = cs.XY(math.cos(angle_rad), math.sin(angle_rad))

    assert motion_makes_goal_progress(s1, s2) == expected


def test_goal_progress_motion_allows_tolerance():
    # Movement that is barely backward due to numerical jitter should be allowed
    s1 = cs.XY(0.0, 0.0)
    s2 = cs.XY(-ov.GOAL_PROGRESS_TOLERANCE / 2, 1.0)

    assert motion_makes_goal_progress(s1, s2)


def test_goal_progress_motion_uses_current_segment_start_to_goal():
    # Movement from an offset segment start toward the goal should be allowed
    s1 = cs.XY(0.0, 1.0)
    s2 = cs.XY(0.5, 0.5)

    assert motion_makes_goal_progress(s1, s2)


def test_goal_progress_motion_rejects_movement_away_from_current_goal_direction():
    # Movement from an offset segment start away from the goal should be rejected
    s1 = cs.XY(0.0, 1.0)
    s2 = cs.XY(-0.5, 1.5)

    assert not motion_makes_goal_progress(s1, s2)
