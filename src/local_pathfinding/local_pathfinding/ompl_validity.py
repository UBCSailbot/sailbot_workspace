from ompl import base
import local_pathfinding.coord_systems as cs

GOAL_PROGRESS_TOLERANCE = 1e-9


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
