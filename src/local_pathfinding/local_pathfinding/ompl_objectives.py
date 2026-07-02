"""Our custom OMPL optimization objectives."""

import math

import numpy as np
from ompl import base as ob
from scipy.interpolate import RegularGridInterpolator

import local_pathfinding.coord_systems as cs
import local_pathfinding.wind_coord_systems as wcs
from local_pathfinding.ompl_validity import (
    get_segment_wind_angle_rad_bc,
    motion_makes_goal_progress,
)

UPWIND_COST_MULTIPLIER = 1.0
DOWNWIND_COST_MULTIPLIER = 1.0
ZERO_SPEED_COST = 1.0
ACCEPTABLE_COST_THRESHOLD = 0.85
WIND_OBJECTIVE_WEIGHT = 0.15
TIME_OBJECTIVE_WEIGHT = 0.85
DEVIATION_OBJECTIVE_WEIGHT = 20
SEGMENT_COUNT_OBJECTIVE_WEIGHT = 0.3
NO_GO_ZONE = math.pi / 4
WIND_COST_SIN_EXPONENT = 80

# Lateral deviation from the start->goal rhumb line saturates the deviation cost to 1.0
# at MAX_DEVIATION_FRACTION * rhumb_length. Smaller values produce a tighter corridor.
MAX_DEVIATION_FRACTION = 0.5
# Higher exponent makes small deviations very cheap and large deviations expensive.
DEVIATION_COST_EXPONENT = 4

# Flat cost charged per path segment. OMPL sums motionCost across segments, so the
# total contribution scales linearly with segment count -- no normalization, no saturation.
# Pushes the planner toward fewer segments when other objectives don't otherwise resolve.
SEGMENT_COST = 1.0


#               Estimated Boat Speeds (kmph) as function of True Wind Speed (kmph)
#                               and the Sailing Angle (deg)
#  __________________________________________________________________________________________
# | True  |                             Sailing Angle (deg)                                  |
# | Wind  |__________________________________________________________________________________|
# | Speed |       |       |       |       |       |       |      |      |      |      |      |
# | (Kmph)|   0°  |  45°  |  50°  |  60°  |  75°  |  90°  | 110° | 120° | 135° | 150° | 180° |
# |_______|_______|_______|_______|_______|_______|_______|______|______|______|______|______|
# |  0.0  |  0.0  |  0.0  |  0.0  |  0.0  |  0.0  |  0.0  |  0.0 |  0.0 |  0.0 |  0.0 |  0.0 |
# | 11.1  |  0.0  |  5.0  |  5.4  |  5.8  |  6.1  |  6.0  |  5.6 |  5.2 |  4.6 |  4.0 |  3.6 |
# | 14.8  |  0.0  |  5.9  |  6.5  |  6.9  |  7.2  |  7.1  |  6.8 |  6.5 |  5.9 |  5.3 |  4.8 |
# | 18.5  |  0.0  |  6.6  |  7.3  |  7.7  |  8.1  |  8.1  |  7.8 |  7.5 |  6.9 |  6.3 |  5.8 |
# | 22.2  |  0.0  |  7.2  |  7.9  |  8.4  |  8.7  |  8.8  |  8.6 |  8.4 |  7.8 |  7.2 |  6.7 |
# | 25.9  |  0.0  |  7.7  |  8.4  |  8.7  |  9.0  |  9.2  |  9.0 |  8.9 |  8.5 |  8.0 |  7.6 |
# | 29.6  |  0.0  |  8.0  |  8.7  |  9.0  |  9.3  |  9.4  |  9.4 |  9.2 |  9.0 |  8.6 |  8.3 |
# | 37.0  |  0.0  |  8.2  |  8.9  |  9.3  |  9.6  | 10.0  | 10.0 |  9.8 |  9.6 |  9.3 |  9.2 |
# | 55.0  |  0.0  |  8.5  |  9.3  |  9.6  | 10.0  | 10.0  | 10.0 | 10.0 | 10.0 | 10.0 | 10.0 |
# | 75.0  |  0.0  |  8.7  |  9.6  |  9.8  | 10.0  | 10.0  | 10.0 | 10.0 | 10.0 | 10.0 | 10.0 |
# -------------------------------------------------------------------------------------------

BOAT_SPEEDS_KMPH = np.array(
    [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 5.0, 5.4, 5.8, 6.1, 6.0, 5.6, 5.2, 4.6, 4.0, 3.6],
        [0.0, 5.9, 6.5, 6.9, 7.2, 7.1, 6.8, 6.5, 5.9, 5.3, 4.8],
        [0.0, 6.6, 7.3, 7.7, 8.1, 8.1, 7.8, 7.5, 6.9, 6.3, 5.8],
        [0.0, 7.2, 7.9, 8.4, 8.7, 8.8, 8.6, 8.4, 7.8, 7.2, 6.7],
        [0.0, 7.7, 8.4, 8.7, 9.0, 9.2, 9.0, 8.9, 8.5, 8.0, 7.6],
        [0.0, 8.0, 8.7, 9.0, 9.3, 9.4, 9.4, 9.2, 9.0, 8.6, 8.3],
        [0.0, 8.2, 8.9, 9.3, 9.6, 10.0, 10.0, 9.8, 9.6, 9.3, 9.2],
        [0.0, 8.5, 9.3, 9.6, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
        [0.0, 8.7, 9.6, 9.8, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
    ]
)

TW_SPEEDS_KMPH_GC = [0.0, 11.1, 14.8, 18.5, 22.2, 25.9, 29.6, 37.0, 55.0, 75.0]
# Absolute angle between the path segment bearing and true wind direction, bounded by [0, 180].
SAILING_ANGLES_DEG_GC = [0, 45, 50, 60, 75, 90, 110, 120, 135, 150, 180]

ESTIMATED_TOP_BOAT_SPEED = np.max(BOAT_SPEEDS_KMPH)


class GoalDirectionObjective(ob.OptimizationObjective):
    """The GoalDirectionObjective assigns an infinite cost to path segments that move away
    from the goal.

    Progress is measured by projecting the segment vector onto a goal direction vector. This
    allows sideways motion, such as tacking or obstacle avoidance, as long as the segment does
    not move backward relative to the goal direction.
    """

    def __init__(
        self,
        space_information: ob.SpaceInformation,
        goal_position_in_xy: cs.XY,
    ) -> None:
        self.goal_position_in_xy = goal_position_in_xy
        ob.OptimizationObjective.__init__(self, space_information)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Defines the cost of a path segment, from s1 to s2, based on whether the segment
           makes progress toward the goal.

        The segment vector is dotted with the direction from s1 to the goal. If the projection
        is negative beyond GOAL_PROGRESS_TOLERANCE, the segment is moving away from the goal
        and receives infinite cost. Otherwise, the segment receives zero additional cost.

        Args:
            s1 (SE2StateInternal): The start of the path segment
            s2 (SE2StateInternal): The end of the path segment

        Returns:
            ob.Cost: The cost of the path segment from s1 to s2
        """

        return (
            ob.Cost(0)
            if motion_makes_goal_progress(s1, s2, self.goal_position_in_xy)
            else ob.Cost(float("inf"))
        )


class WindObjective(ob.OptimizationObjective):
    """Optimization objective that scores path segments by wind alignment.

    The hard wind no-go rejection is handled by `GoalProgressWindMotionValidator`; this objective
    keeps wind alignment in the optimization cost so the planner still prefers better sailing
    angles among valid motions.

    Attributes:
        tw_dir_rad_gc (float): Direction of true wind in global coordinate radians (-pi, pi]
    """

    def __init__(
        self,
        space_information: ob.SpaceInformation,
        tw_dir_rad_gc: float,
    ) -> None:
        super().__init__(space_information)
        self.tw_dir_rad_gc = tw_dir_rad_gc

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Return the wind-alignment cost for the segment from `s1` to `s2`.

        Segments closer to directly upwind or downwind receive higher cost. Segments inside the
        no-go zone receive max wind cost here and are also rejected by the motion validator.

        Args:
            s1 (SE2StateInternal): The starting point of the path segment
            s2 (SE2StateInternal): The ending point of the path segment

        Returns:
            ob.Cost: The cost of travelling along the path segment.
        """
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        return ob.Cost(WindObjective.wind_direction_cost(s1_xy, s2_xy, self.tw_dir_rad_gc))

    @staticmethod
    def wind_direction_cost(s1: cs.XY, s2: cs.XY, tw_dir_rad_gc: float) -> float:
        """Compute a wind-alignment cost for a path segment.

        Args:
            s1 (cs.XY): The start point of the path segment
            s2 (cs.XY): The end point of the path segment
            tw_dir_rad_gc (float): The direction of the true wind in radians, (-pi, pi]

        Returns:
            float: The cost of the path segment from s1 to s2, in the interval [0, 1].
        """
        segment_wind_angle_rad_bc = get_segment_wind_angle_rad_bc(
            s1,
            s2,
            tw_dir_rad_gc,
        )
        if (
            segment_wind_angle_rad_bc <= NO_GO_ZONE
            or segment_wind_angle_rad_bc >= math.pi - NO_GO_ZONE
        ):
            return 1.0
        return math.sin(2 * segment_wind_angle_rad_bc) ** WIND_COST_SIN_EXPONENT


class DeviationObjective(ob.OptimizationObjective):
    """Penalises path segments that swing wide of the start->goal rhumb line.

    Encourages many tight tacks over one large swing, because each segment's cost grows with
    its midpoint's perpendicular distance from the rhumb axis. Segments sitting on the axis
    pay zero, so when the wind allows a straight shot to the goal the objective contributes
    nothing.

    Attributes:
        start_position_in_xy (cs.XY): The path start in the local XY frame
        goal_position_in_xy (cs.XY): The current goal in the local XY frame
    """

    def __init__(
        self,
        space_information: ob.SpaceInformation,
        start_position_in_xy: cs.XY,
        goal_position_in_xy: cs.XY,
    ) -> None:
        super().__init__(space_information)
        self.start_position_in_xy = start_position_in_xy
        self.goal_position_in_xy = goal_position_in_xy

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        return ob.Cost(
            DeviationObjective.deviation_cost(
                s1_xy,
                s2_xy,
                self.start_position_in_xy,
                self.goal_position_in_xy,
            )
        )

    @staticmethod
    def deviation_cost(s1: cs.XY, s2: cs.XY, start: cs.XY, goal: cs.XY) -> float:
        """Return the deviation cost for a segment as a function of its midpoint's
        perpendicular distance from the start->goal rhumb line.

        Args:
            s1 (cs.XY): The start of the path segment
            s2 (cs.XY): The end of the path segment
            start (cs.XY): The path start position
            goal (cs.XY): The current goal position

        Returns:
            float: The deviation cost in the interval [0, 1].
        """
        axis_x = goal.x - start.x
        axis_y = goal.y - start.y
        axis_len = math.hypot(axis_x, axis_y)
        if math.isclose(axis_len, 0.0):
            return 0.0

        mid_x = 0.5 * (s1.x + s2.x)
        mid_y = 0.5 * (s1.y + s2.y)
        offset_x = mid_x - start.x
        offset_y = mid_y - start.y

        perp_dist = abs(offset_x * axis_y - offset_y * axis_x) / axis_len
        max_dist = MAX_DEVIATION_FRACTION * axis_len
        if math.isclose(max_dist, 0.0):
            return 0.0

        normalized = perp_dist / max_dist
        return min(1.0, normalized**DEVIATION_COST_EXPONENT)


class SegmentCountObjective(ob.OptimizationObjective):
    """Charges a fixed cost per path segment, irrespective of segment geometry.

    OMPL sums motionCost across segments when scoring a path, so the contribution of this
    objective grows linearly with segment count. Unlike normalized objectives, there is no
    saturation: more segments always costs proportionally more, giving the planner a sharp
    signal to consolidate when other objectives are indifferent.
    """

    def __init__(self, space_information: ob.SpaceInformation) -> None:
        super().__init__(space_information)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        return ob.Cost(SEGMENT_COST)


class TimeObjective(ob.OptimizationObjective):
    """The Time Objective assigns a cost, to any path segment, that is proportional to the
    estimated time it will take for the boat to travel from the start of the segment to the
    end of the segment.

    Attributes:
        tw_dir_rad_gc (float): The direction of wind in global coordinate radians (-pi, pi]
        tw_speed_kmph (float): The speed of the true wind in km/h
    """

    interpolation = RegularGridInterpolator(
        (TW_SPEEDS_KMPH_GC, SAILING_ANGLES_DEG_GC),
        BOAT_SPEEDS_KMPH,
        bounds_error=False,  # no error on out of bounds call
        # returns max speed for any input outside the range of the table
        fill_value=ESTIMATED_TOP_BOAT_SPEED,
    )

    def __init__(
        self,
        space_information: ob.SpaceInformation,
        tw_dir_rad_gc: float,
        tw_speed_kmph: float,
    ) -> None:
        super().__init__(space_information)
        self.tw_dir_rad_gc = tw_dir_rad_gc
        self.tw_speed_kmph = tw_speed_kmph

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Defines the cost of a path segment, from s1 to s2, as the estimated time it will take
           the boat to travel in a straight line from s1 to s2.

        Args:
            s1 (SE2StateInternal): The start of the path segment
            s2 (SE2StateInternal): The end of the path segment

        Returns:
            ob.Cost: The cost of the path segment from s1 to s2
        """

        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        return ob.Cost(
            TimeObjective.time_cost(
                s1_xy,
                s2_xy,
                self.tw_dir_rad_gc,
                self.tw_speed_kmph,
            )
        )

    @staticmethod
    def time_cost(
        s1: cs.XY,
        s2: cs.XY,
        tw_dir_rad_gc: float,
        tw_speed_kmph: float,
    ) -> float:
        """Returns a cost proportional to the estimated amount of time it will take for the boat
           to travel from s1 to s2.

        Args:
            s1 (cs.XY): The start point of the path segment
            s2 (cs.XY): The end point of the path segment
            tw_dir_rad_gc (float): The direction of wind in global coord radians (-pi, pi]
            tw_speed_kmph (float): The true wind speed in km/h

        Returns:
            float: The cost the path segment from s1 to s2, in the interval [0, 1]

        """
        path_segment_true_bearing_radians = cs.get_path_segment_true_bearing(s1, s2, rad=True)

        sailbot_speed = TimeObjective.get_sailbot_speed(
            path_segment_true_bearing_radians,
            tw_dir_rad_gc,
            tw_speed_kmph,
        )

        # exit early to avoid dividing by sailbot_speed when it's close to 0
        if math.isclose(sailbot_speed, 0):
            return ZERO_SPEED_COST

        distance = math.hypot(s2.y - s1.y, s2.x - s1.x)
        time = distance / sailbot_speed
        ideal_time = distance / ESTIMATED_TOP_BOAT_SPEED
        deltaT = time - ideal_time
        # This normalizes the time to an interval of (0,1) more efficiently than sigmoid
        normalized_time = deltaT / (1 + abs(deltaT))
        return normalized_time

    @staticmethod
    def get_sailbot_speed(
        path_segment_true_bearing_rad: float,
        tw_dir_rad_gc: float,
        tw_speed_kmph: float,
    ) -> float:

        tw_angle_rad_bc = abs(
            wcs.get_true_wind_angle(path_segment_true_bearing_rad, tw_dir_rad_gc)
        )

        # this bounds the twa to a range of 0 to 180 degrees
        # we can take the absolute value because we don't care if the wind is blowing
        # on the port or starboard side when it comes to calculating the estimated speed
        # and having the twa in the range of [0, 180] means we don't have to cover negative
        # twa values in the BOAT_SPEEDS table
        tw_angle_deg_gc = abs(cs.bound_to_180(math.degrees(tw_angle_rad_bc)))

        # since the twa is bounded to [0, 180], the only time the interpolator would need to
        # use the fill_value is if the tw_speed_kmph is greater than the max accounted for
        # in the BOAT_SPEEDS table, in which case the interpolator will return it's configured
        # fill_value (see interpolation definition at top of class)
        return TimeObjective.interpolation((tw_speed_kmph, tw_angle_deg_gc))


def get_sailing_objective(
    space_information: ob.SpaceInformation,
    tw_dir_deg_gc: float,
    tw_speed_kmph: float,
    start_position_in_xy: cs.XY,
    goal_position_in_xy: cs.XY,
) -> ob.OptimizationObjective:
    """Build the combined sailing optimization objective for the current wind snapshot.

    True wind is converted to radians once, then shared by the wind and time objectives.
    Goal direction remains in the objective stack even though goal progress is also enforced as a
    hard motion-validity check.
    """

    tw_dir_rad_gc = math.radians(tw_dir_deg_gc)
    multiObjective = ob.MultiOptimizationObjective(si=space_information)
    multiObjective.addObjective(
        objective=WindObjective(
            space_information,
            tw_dir_rad_gc,
        ),
        weight=WIND_OBJECTIVE_WEIGHT,
    )
    multiObjective.addObjective(
        objective=TimeObjective(space_information, tw_dir_rad_gc, tw_speed_kmph),
        weight=TIME_OBJECTIVE_WEIGHT,
    )
    multiObjective.addObjective(
        objective=DeviationObjective(space_information, start_position_in_xy, goal_position_in_xy),
        weight=DEVIATION_OBJECTIVE_WEIGHT,
    )
    multiObjective.addObjective(
        objective=SegmentCountObjective(space_information),
        weight=SEGMENT_COUNT_OBJECTIVE_WEIGHT,
    )
    multiObjective.addObjective(
        objective=GoalDirectionObjective(space_information, goal_position_in_xy),
        weight=1.0,  # should always be 1.0
    )
    # this allows the objective to be satisfied once a path with a cost
    # below the threshold has been found
    # this can prevent the solver from running until the time limit in some cases
    multiObjective.setCostThreshold(ACCEPTABLE_COST_THRESHOLD)

    return multiObjective
