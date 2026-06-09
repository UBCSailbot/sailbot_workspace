import math

from ompl import base

import local_pathfinding.coord_systems as cs
import local_pathfinding.wind_coord_systems as wcs

GOAL_PROGRESS_TOLERANCE = 1e-9
NO_GO_ZONE = math.pi / 4


class GoalProgressMotion():
    """Shared base class for checking whether a motion segment progresses toward the goal.

    This class stores the goal position and provides the shared
    `_motion_makes_goal_progress` helper used by:
        - GoalProgressMotionValidator, which rejects invalid OMPL motions during planning.
        - GoalDirectionObjective, which gives non-progressing motions infinite cost.

    A motion is considered valid for goal progress when the segment from `s1` to `s2` has a
    non-negative projection onto the vector from `s1` to the goal. This still permits sideways
    motion, such as tacking or obstacle avoidance, as long as the motion does not go backward
    relative to the goal.
    """

    def __init__(self, goal_position_in_xy):
        self.goal_position_in_xy = goal_position_in_xy

    def _motion_makes_goal_progress(self, s1, s2) -> bool:
        s12_vec = (s2.getX() - s1.getX(), s2.getY() - s1.getY())
        s1g_vec = (
            self.goal_position_in_xy.x - s1.getX(),
            self.goal_position_in_xy.y - s1.getY(),
        )

        s12_s1g_dot = s12_vec[0] * s1g_vec[0] + s12_vec[1] * s1g_vec[1]
        return s12_s1g_dot >= -GOAL_PROGRESS_TOLERANCE


def get_segment_wind_angle_rad_bc(s1: cs.XY, s2: cs.XY, tw_direction_rad_gc: float) -> float:
    """Return the absolute angle between a segment bearing and true wind direction.

    Args:
        s1 (cs.XY): The start point of the path segment
        s2 (cs.XY): The end point of the path segment
        tw_direction_rad_gc (float): The direction of the true wind in radians, (-pi, pi]

    Returns:
        float: The wind-relative segment angle in boat-coordinate radians, [0, pi].
    """
    segment_true_bearing_rad = cs.get_path_segment_true_bearing(s1, s2, rad=True)
    return abs(wcs.get_true_wind_angle(segment_true_bearing_rad, tw_direction_rad_gc))


def in_wind_no_go_zone(s1: cs.XY, s2: cs.XY, tw_direction_rad_gc: float) -> bool:
    """Check whether a segment points too close to directly upwind or downwind.

    A segment is in the no-go zone when its wind-relative angle is <= NO_GO_ZONE or
    >= pi - NO_GO_ZONE, i.e. within 45 degrees of directly upwind or downwind.

    Args:
        s1 (cs.XY): The start point of the path segment
        s2 (cs.XY): The end point of the path segment
        tw_direction_rad_gc (float): The direction of the true wind in radians, (-pi, pi]

    Returns:
        bool: True if the segment is in the wind no-go zone, else False.
    """
    segment_wind_angle_rad_bc = get_segment_wind_angle_rad_bc(s1, s2, tw_direction_rad_gc)

    return (
        segment_wind_angle_rad_bc <= NO_GO_ZONE
        or segment_wind_angle_rad_bc >= math.pi - NO_GO_ZONE
    )


class GoalProgressMotionValidator(base.MotionValidator, GoalProgressMotion):
    """OMPL motion validator that rejects motions that do not progress toward the goal.

    This class inherits from `base.MotionValidator` so it can be installed on OMPL's
    SpaceInformation, and from `GoalProgressMotion` for the shared goal-progress test. It first
    rejects segments that move away from the goal, then delegates to OMPL's
    `DiscreteMotionValidator` for the usual interpolated collision and state-validity checks.
    """

    def __init__(self, space_information, goal_position_in_xy: cs.XY):
        super().__init__(space_information)
        self.space_information = space_information
        self.goal_position_in_xy = goal_position_in_xy
        self.default_motion_validator = base.DiscreteMotionValidator(space_information)

    def checkMotion(self, s1, s2, *args) -> bool:
        """Check collision validity and goal progress for a motion."""
        if not self._motion_makes_goal_progress(s1, s2):
            return False

        return self.default_motion_validator.checkMotion(s1, s2, *args)


class WindMotionValidator(base.MotionValidator):
    """OMPL motion validator that rejects motions in the wind no-go zone.

    This class inherits from `base.MotionValidator` so it can be installed on OMPL's
    SpaceInformation. It rejects segments that point within the configured wind no-go zone, then
    delegates to OMPL's `DiscreteMotionValidator` for the usual interpolated collision and
    state-validity checks.
    """

    def __init__(
        self,
        space_information,
        boat_heading_deg_gc: float,
        boat_speed_kmph: float,
        aw_direction_deg_bc: float,
        aw_speed_kmph: float,
    ):
        super().__init__(space_information)
        aw_direction_deg_gc = wcs.boat_to_global_coordinate(
            boat_heading_deg_gc,
            aw_direction_deg_bc,
        )
        tw_direction_rad_gc, _ = wcs.get_true_wind(
            aw_direction_deg_gc,
            aw_speed_kmph,
            boat_heading_deg_gc,
            boat_speed_kmph,
        )
        self.tw_direction_rad_gc = tw_direction_rad_gc
        self.default_motion_validator = base.DiscreteMotionValidator(space_information)

    def checkMotion(self, s1, s2, *args) -> bool:
        """Check wind feasibility and collision validity for a motion."""
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        if in_wind_no_go_zone(s1_xy, s2_xy, self.tw_direction_rad_gc):
            return False

        return self.default_motion_validator.checkMotion(s1, s2, *args)
