"""Our custom OMPL optimization objectives."""

import math

import numpy as np
from ompl import base as ob
from scipy.interpolate import RegularGridInterpolator

import local_pathfinding.coord_systems as cs
import local_pathfinding.wind_coord_systems as wcs

UPWIND_COST_MULTIPLIER = 1.0
DOWNWIND_COST_MULTIPLIER = 1.0
ZERO_SPEED_COST = 1.0
ACCEPTABLE_COST_THRESHOLD = 0.0
WIND_OBJECTIVE_WEIGHT = 1.0
SPEED_OBJECTIVE_WEIGHT = 1.0


#       Estimated Boat Speeds (kmph) as function of True Wind Speed (kmph)
#                         and the Sailing Angle (deg)
#  ___________________________________________________________________________
# |                   |              Sailing Angle (deg)                      |
# |                   |_______________________________________________________|
# |                   |   0°   |  20°  |  30°  |  45°  |  90°  | 135°  | 180° |
# |___________________|_______________________________________________________|
# |           |  0.0  |  0.0   |  0.0  |  0.0  |  0.0  |  0.0  |  0.0  |  0.0 |
# | True Wind |  9.3  |  0.0   |  0.0  |  0.4  |  1.1  |  3.2  |  3.7  |  2.8 |
# |  Speed    | 18.5  |  0.0   |  0.3  |  1.9  |  3.7  |  9.3  | 13.0  |  9.2 |
# |  (kmph)   | 27.8  |  0.0   |  0.9  |  3.7  |  7.4  | 14.8  | 18.5  | 13.0 |
# |           | 37.0  |  0.0   |  1.3  |  5.6  |  9.3  | 18.5  | 24.1  | 18.5 |
# ----------------------------------------------------------------------------

BOAT_SPEEDS = np.array(
    [
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0.4, 1.1, 3.2, 3.7, 2.8],
        [0, 0.3, 1.9, 3.7, 7.5, 10.0, 9.2],
        [0, 0.9, 3.7, 7.4, 10.0, 10.0, 10.0],
        [0, 1.3, 5.6, 9.3, 10.0, 10.0, 10.0],
    ]
)
TRUE_WIND_SPEEDS = [0, 9.3, 18.5, 27.8, 37.0]
SAILING_ANGLES = [0, 20, 30, 45, 90, 135, 180]

ESTIMATED_TOP_BOAT_SPEED = np.max(BOAT_SPEEDS)


class WindObjective(ob.OptimizationObjective):
    """The WindObjective assigns a high cost to any path segment which is oriented directly
    (or almost directly) upwind or downwind.

    Attributes:
        true_wind_direction (float): The direction of the true wind in radians (-pi, pi]
    """

    def __init__(
        self,
        space_information,
        true_wind_direction_radians: float,
    ):
        super().__init__(space_information)
        self.true_wind_direction_radians = true_wind_direction_radians

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Defines the cost of a path segment, from s1 to s2, with regards to the direction of the
        segment and the wind. The closer the segment is to pointing upwind or downwind the higher
        the motion cost.

        Args:
            s1 (SE2StateInternal): The starting point of the path segment
            s2 (SE2StateInternal): The ending point of the path segment

        Returns:
            ob.Cost: The cost of travelling along the path segment.
        """
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        return ob.Cost(
            WindObjective.wind_direction_cost(s1_xy, s2_xy, self.true_wind_direction_radians)
        )

    @staticmethod
    def wind_direction_cost(s1: cs.XY, s2: cs.XY, true_wind_direction_radians: float) -> float:
        """Returns a high cost when the path segment from s1 to s2 is pointing directly
           (or close to directly) upwind or downwind.

        Args:
            s1 (cs.XY): The start point of the path segment
            s2 (cs.XY): The end point of the path segment
            true_wind_direction_radians (float): The direction of the true wind in
            radians (-pi, pi]

        Returns:
            float: The cost the path segment from s1 to s2, in the interval [0, 1]
        """
        segment_heading_radians = cs.get_path_segment_true_bearing(s1, s2, rad=True)
        angle_diff_radians = segment_heading_radians - true_wind_direction_radians
        cos_angle = math.cos(angle_diff_radians)

        # The target point of sail (POS) is a beam reach for max speed and stability
        # A beam reach is when the true wind direction is perpendicular to the heading of the boat
        # That is why we assign the min cost of 0 to any segment that corresponds to a beam reach
        if cos_angle > 0:
            return UPWIND_COST_MULTIPLIER * cos_angle
        else:
            return DOWNWIND_COST_MULTIPLIER * abs(cos_angle)


class TimeObjective(ob.OptimizationObjective):
    """The Time Objective assigns a cost, to any path segment, that is proportional to the
    estimated time it will take for the boat to travel from the start of the segment to the
    end of the segment.

    Attributes:
        wind_direction (float): The direction of the wind in radians (-pi, pi]
        wind_speed (float): The speed of the wind in m/s
    """

    interpolation = RegularGridInterpolator(
        (TRUE_WIND_SPEEDS, SAILING_ANGLES),
        BOAT_SPEEDS,
        bounds_error=False,  # no error on out of bounds call
        fill_value=0,  # returns 0 for any input outside the range of the table
    )

    def __init__(
        self,
        space_information,
        true_wind_direction_radians: float,
        true_wind_speed_kmph: float,
    ):
        super().__init__(space_information)
        self.true_wind_direction_radians = true_wind_direction_radians
        self.true_wind_speed_kmph = true_wind_speed_kmph

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
                self.true_wind_direction_radians,
                self.true_wind_speed_kmph,
            )
        )

    @staticmethod
    def time_cost(
        s1: cs.XY, s2: cs.XY, true_wind_direction_radians: float, true_wind_speed_kmph
    ) -> float:
        """Returns a cost proportional to the estimated amount of time it will take for the boat
           to travel from s1 to s2.

        Args:
            s1 (cs.XY): The start point of the path segment
            s2 (cs.XY): The end point of the path segment
            true_wind_direction_radians (float): The direction of the true wind in
            radians (-pi, pi]
            true_wind_speed_kmph (float): The true wind speed in km/h

        Returns:
            float: The cost the path segment from s1 to s2, in the interval [0, 1]

        """
        path_segment_true_bearing_radians = cs.get_path_segment_true_bearing(s1, s2, rad=True)

        sailbot_speed = TimeObjective.get_sailbot_speed(
            path_segment_true_bearing_radians,
            true_wind_direction_radians,
            true_wind_speed_kmph,
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
        path_segment_true_bearing_radians: float,
        true_wind_direction_radians: float,
        true_wind_speed_kmph: float,
    ) -> float:
        sailing_angle_radians = abs(
            path_segment_true_bearing_radians - true_wind_direction_radians
        )
        # the sailing angle can always be represented by a positive value between 0 and 180 degrees
        sailing_angle_degrees = abs(cs.bound_to_180(math.degrees(sailing_angle_radians)))

        return TimeObjective.interpolation((true_wind_speed_kmph, sailing_angle_degrees))


class MinimumTurnsObjective(ob.OptimizationObjective):
    """The Minimum Turns Objective assigns a cost, to any path segment with a turn in it that is
    proportional to the magnitude of the turn. A turn is detected when the yaw of s1 != yaw of s2.
    The magnitude of the turn is defined as |(yaw of s1) - (yaw of s2)|.
    """

    def __init__(self, space_information):
        super().__init__(space_information)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        yaw1_radians = cs.bound_to_180(s1.get().getYaw(), rad=True)
        yaw2_radians = cs.bound_to_180(s2.get().getYaw(), rad=True)
        return ob.Cost(MinimumTurnsObjective.turn_cost(yaw1_radians, yaw2_radians))

    @staticmethod
    def turn_cost(yaw1_radians: float, yaw2_radians: float) -> float:
        """This function returns a cost proportional to the size of the acute angle between
        yaw1_radians and yaw2_radians.

        Args:
            yaw1_radians (float): the yaw of state 1 in (-pi, pi]
            yaw2_radians (float): the yaw of state 2 in (-pi, pi]

        Returns:
            float: the cost of the turning from yaw1 to yaw2, in the interval [0, 1]
        """
        return abs(cs.bound_to_180(yaw2_radians - yaw1_radians, rad=True)) / np.pi


def get_sailing_objective(
    space_information,
    simple_setup,
    boat_heading_degrees: float,
    boat_speed_kmph: float,
    apparent_wind_direction_degrees: float,
    apparent_wind_speed_kmph: float,
) -> ob.OptimizationObjective:

    apparent_wind_direction_degrees_global_coordinates = wcs.boat_to_global_coordinate(
        boat_heading_degrees, apparent_wind_direction_degrees
    )

    tw_dir_rad, tw_speed_kmph = wcs.get_true_wind(
        apparent_wind_direction_degrees_global_coordinates,
        apparent_wind_speed_kmph,
        boat_heading_degrees,
        boat_speed_kmph,
    )

    multiObjective = ob.MultiOptimizationObjective(si=space_information)
    multiObjective.addObjective(
        objective=WindObjective(
            space_information,
            tw_dir_rad,
        ),
        weight=WIND_OBJECTIVE_WEIGHT,
    )
    multiObjective.addObjective(
        objective=TimeObjective(
            space_information, tw_dir_rad, tw_speed_kmph
        ),
        weight=SPEED_OBJECTIVE_WEIGHT,
    )
    # this allows the objective to be satisfied once a path with a cost
    # below the threshold has been found
    # this can prevent the solver from running until the time limit in some cases
    multiObjective.setCostThreshold(ACCEPTABLE_COST_THRESHOLD)

    return multiObjective
