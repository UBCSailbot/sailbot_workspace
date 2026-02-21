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
TIME_OBJECTIVE_WEIGHT = 1.0
NO_GO_ZONE = math.pi / 4
WIND_COST_SIN_EXPONENT = 80

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

BOAT_SPEEDS = np.array(
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

TRUE_WIND_SPEEDS = [0.0, 11.1, 14.8, 18.5, 22.2, 25.9, 29.6, 37.0, 55.0, 75.0]
SAILING_ANGLES = [0, 45, 50, 60, 75, 90, 110, 120, 135, 150, 180]

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
    def wind_direction_cost(s1: cs.XY, s2: cs.XY, tw_direction_rad: float) -> float:
        """Returns a high cost when the path segment from s1 to s2 is pointing directly
           (or close to directly) upwind or downwind.

        Args:
            s1 (cs.XY): The start point of the path segment
            s2 (cs.XY): The end point of the path segment
            tw_direction_rad (float): The direction of the true wind in radians, (-pi, pi]

        Returns:
            float: The cost the path segment from s1 to s2, in the interval [0, 1]
        """
        segment_true_bearing_rad = cs.get_path_segment_true_bearing(s1, s2, rad=True)
        tw_angle_rad = abs(wcs.get_true_wind_angle(segment_true_bearing_rad, tw_direction_rad))
        if tw_angle_rad > NO_GO_ZONE or tw_angle_rad < math.pi - NO_GO_ZONE:
            cost = math.sin(2*tw_angle_rad) ** WIND_COST_SIN_EXPONENT
            return cost
        return 1.0


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
        # returns max speed for any input outside the range of the table
        fill_value=ESTIMATED_TOP_BOAT_SPEED,
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
        path_segment_true_bearing_rad: float,
        tw_direction_rad: float,
        tw_speed_kmph: float,
    ) -> float:

        tw_angle_rad = abs(
            wcs.get_true_wind_angle(path_segment_true_bearing_rad, tw_direction_rad)
        )

        # this bounds the twa to a range of 0 to 180 degrees
        # we can take the absolute value because we don't care if the wind is blowing
        # on the port or starboard side when it comes to calculating the estimated speed
        # and having the twa in the range of [0, 180] means we don't have to cover negative
        # twa values in the BOAT_SPEEDS table
        tw_angle_deg = abs(cs.bound_to_180(math.degrees(tw_angle_rad)))

        # since the twa is bounded to [0, 180], the only time the interpolator would need to
        # use the fill_value is if the tw_speed_kmph is greater than the max accounted for
        # in the BOAT_SPEEDS table, in which case the interpolator will return it's configured
        # fill_value (see interpolation definition at top of class)
        return TimeObjective.interpolation((tw_speed_kmph, tw_angle_deg))


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
        objective=TimeObjective(space_information, tw_dir_rad, tw_speed_kmph),
        weight=TIME_OBJECTIVE_WEIGHT,
    )
    # this allows the objective to be satisfied once a path with a cost
    # below the threshold has been found
    # this can prevent the solver from running until the time limit in some cases
    multiObjective.setCostThreshold(ACCEPTABLE_COST_THRESHOLD)

    return multiObjective
