"""Motion-validity helpers and validators for OMPL path planning."""

from __future__ import annotations

import math
from typing import Any

from ompl import base as ob

import local_pathfinding.coord_systems as cs
import local_pathfinding.wind_coord_systems as wcs

GOAL_PROGRESS_TOLERANCE = 1e-9
NO_GO_ZONE = math.pi / 4


def motion_makes_goal_progress(
    s1: ob.SE2StateInternal,
    s2: ob.SE2StateInternal,
    goal_position_in_xy: cs.XY,
) -> bool:
    """Return whether a motion segment moves toward or sideways relative to the goal.

    A segment is considered valid when its projection onto the vector from `s1` to the goal is
    non-negative within `GOAL_PROGRESS_TOLERANCE`. This permits sideways tacking and obstacle
    avoidance while rejecting backward motion relative to the goal.
    """
    s12_vec = (s2.getX() - s1.getX(), s2.getY() - s1.getY())
    s1g_vec = (
        goal_position_in_xy.x - s1.getX(),
        goal_position_in_xy.y - s1.getY(),
    )

    s12_s1g_dot = s12_vec[0] * s1g_vec[0] + s12_vec[1] * s1g_vec[1]
    return s12_s1g_dot >= -GOAL_PROGRESS_TOLERANCE


def get_segment_wind_angle_rad_bc(s1: cs.XY, s2: cs.XY, tw_dir_rad_gc: float) -> float:
    """Return the absolute angle between a segment bearing and true wind direction.

    Args:
        s1 (cs.XY): The start point of the path segment
        s2 (cs.XY): The end point of the path segment
        tw_dir_rad_gc (float): The direction of the true wind in radians, (-pi, pi]

    Returns:
        float: The wind-relative segment angle in boat-coordinate radians, [0, pi].
    """
    segment_true_bearing_rad = cs.get_path_segment_true_bearing(s1, s2, rad=True)
    return abs(wcs.get_true_wind_angle(segment_true_bearing_rad, tw_dir_rad_gc))


def in_wind_no_go_zone(s1: cs.XY, s2: cs.XY, tw_dir_rad_gc: float) -> bool:
    """Check whether a segment points too close to directly upwind or downwind.

    A segment is in the no-go zone when its wind-relative angle is <= NO_GO_ZONE or
    >= pi - NO_GO_ZONE, i.e. within 45 degrees of directly upwind or downwind.

    Args:
        s1 (cs.XY): The start point of the path segment
        s2 (cs.XY): The end point of the path segment
        tw_dir_rad_gc (float): The direction of the true wind in radians, (-pi, pi]

    Returns:
        bool: True if the segment is in the wind no-go zone, else False.
    """
    segment_wind_angle_rad_bc = get_segment_wind_angle_rad_bc(s1, s2, tw_dir_rad_gc)

    return (
        segment_wind_angle_rad_bc <= NO_GO_ZONE
        or segment_wind_angle_rad_bc >= math.pi - NO_GO_ZONE
    )


class GoalProgressWindMotionValidator(ob.MotionValidator):
    """OMPL motion validator that enforces goal progress and wind feasibility.

    OMPL exposes one installed motion-validator slot, so this validator composes the hard
    goal-progress and wind no-go checks before delegating once to OMPL's
    `DiscreteMotionValidator` for interpolated collision and state-validity checks.
    """

    def __init__(
        self,
        space_information: ob.SpaceInformation,
        goal_position_in_xy: cs.XY,
        tw_dir_deg_gc: float,
    ) -> None:
        ob.MotionValidator.__init__(self, space_information)
        tw_dir_rad_gc = math.radians(tw_dir_deg_gc)
        self.tw_dir_rad_gc = tw_dir_rad_gc
        self.space_information = space_information
        self.goal_position_in_xy = goal_position_in_xy
        self.default_motion_validator = ob.DiscreteMotionValidator(space_information)

    def checkMotion(
        self,
        s1: ob.SE2StateInternal,
        s2: ob.SE2StateInternal,
        *args: Any,
    ) -> bool:
        """Check goal progress, wind feasibility, and discrete collision validity."""
        if not motion_makes_goal_progress(s1, s2, self.goal_position_in_xy):
            return False

        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        if in_wind_no_go_zone(s1_xy, s2_xy, self.tw_dir_rad_gc):
            return False

        return self.default_motion_validator.checkMotion(s1, s2, *args)
