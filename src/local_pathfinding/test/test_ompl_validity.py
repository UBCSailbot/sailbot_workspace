import math

from ompl import base
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
    return ov.motion_makes_goal_progress(
        MockState(s1.x, s1.y),
        MockState(s2.x, s2.y),
        goal_position_in_xy,
    )


def make_space_information():
    space = base.SE2StateSpace()
    bounds = base.RealVectorBounds(2)
    bounds.setLow(0, -2.0)
    bounds.setLow(1, -2.0)
    bounds.setHigh(0, 2.0)
    bounds.setHigh(1, 2.0)
    space.setBounds(bounds)

    space_information = base.SpaceInformation(space)
    space_information.setStateValidityChecker(base.StateValidityCheckerFn(lambda state: True))
    space_information.setup()
    return space, space_information


def make_state(space, x: float, y: float):
    state = base.State(space)
    state().setXY(x, y)
    return state


def make_goal_progress_wind_motion_validator(
    space_information,
    goal_position_in_xy: cs.XY,
):
    return ov.GoalProgressWindMotionValidator(
        space_information=space_information,
        goal_position_in_xy=goal_position_in_xy,
        tw_dir_deg_gc=0.0,
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
            # Movement -91 degrees to the goal direction is slightly backward
            -91.0,
            False,
        ),
        (
            # Movement 270 (negative 90 degrees direction) to the goal direction should be allowed
            270.0,
            True,
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


@pytest.mark.parametrize(
    "s1, s2, tw_dir_rad_gc, expected",
    [
        (
            # Segment points directly into the wind
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            0.0,
            0.0,
        ),
        (
            # Segment is perpendicular to the wind
            cs.XY(0.0, 0.0),
            cs.XY(1.0, 0.0),
            0.0,
            math.pi / 2,
        ),
        (
            # Segment points directly downwind
            cs.XY(0.0, 0.0),
            cs.XY(0.0, -1.0),
            0.0,
            math.pi,
        ),
    ],
)
def test_get_segment_wind_angle_rad_bc(
    s1: cs.XY,
    s2: cs.XY,
    tw_dir_rad_gc: float,
    expected: float,
):
    assert ov.get_segment_wind_angle_rad_bc(s1, s2, tw_dir_rad_gc) == pytest.approx(
        expected
    )


@pytest.mark.parametrize(
    "s1, s2, tw_dir_rad_gc, expected",
    [
        (
            # Wind direction and segment both point north
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            0.0,
            True,
        ),
        (
            # Wind direction south and segment points north
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            math.pi,
            True,
        ),
        (
            # Exactly on the upwind no-go boundary
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            ov.NO_GO_ZONE,
            True,
        ),
        (
            # Exactly on the negative upwind no-go boundary
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            -ov.NO_GO_ZONE,
            True,
        ),
        (
            # Exactly on the downwind no-go boundary
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            math.pi - ov.NO_GO_ZONE,
            True,
        ),
        (
            # Segment points south with wind on the positive downwind boundary
            cs.XY(0.0, 0.0),
            cs.XY(0.0, -1.0),
            math.pi - ov.NO_GO_ZONE,
            True,
        ),
        (
            # Segment points south with wind on the negative downwind boundary
            cs.XY(0.0, 0.0),
            cs.XY(0.0, -1.0),
            -math.pi + ov.NO_GO_ZONE,
            True,
        ),
        (
            # Segment points east, just outside the no-go zone
            cs.XY(0.0, 0.0),
            cs.XY(1.0, 0.0),
            ov.NO_GO_ZONE - 0.0001,
            False,
        ),
        (
            # Segment points west, wind points north
            cs.XY(0.0, 0.0),
            cs.XY(-1.0, 0.0),
            0.0,
            False,
        ),
        (
            # Segment points west, wind points south
            cs.XY(0.0, 0.0),
            cs.XY(-1.0, 0.0),
            math.pi,
            False,
        ),
        (
            # Segment points north, wind at 60 degrees
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            math.pi / 3,
            False,
        ),
        (
            # Segment points north, wind at -55 degrees
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            -0.959931,
            False,
        ),
        (
            # Segment points north, wind at 50 degrees
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            0.872665,
            False,
        ),
    ],
)
def test_in_wind_no_go_zone(
    s1: cs.XY,
    s2: cs.XY,
    tw_dir_rad_gc: float,
    expected: bool,
):
    assert ov.in_wind_no_go_zone(s1, s2, tw_dir_rad_gc) is expected


@pytest.mark.parametrize(
    "s1_xy, s2_xy, goal_position_in_xy, expected",
    [
        (
            # In irons and not making progress
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            cs.XY(0.0, -1.0),
            False,
        ),
        (
            # In irons but making progress
            cs.XY(0.0, 0.0),
            cs.XY(0.0, 1.0),
            cs.XY(0.0, 1.0),
            False,
        ),
        (
            # Not in irons but not making progress
            cs.XY(0.0, 0.0),
            cs.XY(1.0, 0.0),
            cs.XY(-1.0, 0.0),
            False,
        ),
        (
            # Not in irons and making progress
            cs.XY(0.0, 0.0),
            cs.XY(1.0, 0.0),
            cs.XY(1.0, 0.0),
            True,
        ),
    ],
)
def test_goal_progress_wind_motion_validator(
    s1_xy: cs.XY,
    s2_xy: cs.XY,
    goal_position_in_xy: cs.XY,
    expected: bool,
):
    space, space_information = make_space_information()
    validator = make_goal_progress_wind_motion_validator(
        space_information,
        goal_position_in_xy,
    )
    s1 = make_state(space, s1_xy.x, s1_xy.y)
    s2 = make_state(space, s2_xy.x, s2_xy.y)

    assert validator.checkMotion(s1(), s2()) is expected
