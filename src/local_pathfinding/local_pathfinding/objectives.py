"""Our custom OMPL optimization objectives."""

import math
from typing import Tuple

import custom_interfaces.msg as ci
import numpy as np
from ompl import base as ob

import local_pathfinding.coord_systems as cs
from local_pathfinding.coord_systems import bound_to_180

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0

# Upwind downwind constants
HIGHEST_UPWIND_ANGLE_RADIANS = math.radians(40.0)
LOWEST_DOWNWIND_ANGLE_RADIANS = math.radians(20.0)


BOATSPEEDS = np.array(
    [
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0.4, 1.1, 3.2, 3.7, 2.8],
        [0, 0.3, 1.9, 3.7, 9.3, 13.0, 9.2],
        [0, 0.9, 3.7, 7.4, 14.8, 18.5, 13.0],
        [0, 1.3, 5.6, 9.3, 18.5, 24.1, 18.5],
    ]
)

WINDSPEEDS = [0, 9.3, 18.5, 27.8, 37.0]  # The row labels
ANGLES = [0, 20, 30, 45, 90, 135, 180]  # The column labels


def get_true_wind(
    apparent_wind_direction: float,
    apparent_wind_speed: float,
    heading_degrees: float,
    boat_speed_over_ground: float,
) -> Tuple[float, float]:
    """Calculates the true wind direction based on the boat's heading and speed.
    Args:
        apparent_wind_direction (float): The direction of the wind in degrees (-180, 180]. This
        is the apparent wind derived from the wind sensor
        apparent_wind_speed (float): The speed of the wind in kmph. This is the apparent wind
        derived from the wind sensor
        heading_degrees (float): The heading of the boat in degrees (-180, 180]. This is
        derived from the GPS
        speed (float): The speed of the boat in kmph. This is derived from the GPS.
    Returns:
        float: The true wind direction in radians (-pi, pi]
    """
    wind_radians = math.radians(apparent_wind_direction)

    # boat wind is in the direction of the boat heading
    boat_wind_radians = math.radians(bound_to_180(heading_degrees + 180))

    apparent_wind_east = apparent_wind_speed * math.sin(wind_radians)
    apparent_wind_north = apparent_wind_speed * math.cos(wind_radians)

    boat_wind_east = boat_speed_over_ground * math.sin(boat_wind_radians)
    boat_wind_north = boat_speed_over_ground * math.cos(boat_wind_radians)

    true_east = apparent_wind_east - boat_wind_east
    true_north = apparent_wind_north - boat_wind_north

    return (math.atan2(true_east, true_north), math.hypot(true_north, true_east))


class Objective(ob.StateCostIntegralObjective):
    """All of our optimization objectives inherit from this class.

    Notes:
    - This class inherits from the OMPL class StateCostIntegralObjective:
        https://ompl.kavrakilab.org/classompl_1_1base_1_1StateCostIntegralObjective.html
    - Camelcase is used for functions that override OMPL functions, as that is their convention.

    Attributes:
        space_information (StateSpacePtr): Contains all the information about
            the space planning is done in.
    """

    def __init__(self, space_information):
        super().__init__(si=space_information, enableMotionCostInterpolation=True)
        self.space_information = space_information

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        raise NotImplementedError


class DistanceObjective(Objective):
    """Generates a distance objective function

    Attributes:
        method (DistanceMethod): The method of the distance objective function
        ompl_path_objective (ob.PathLengthOptimizationObjective): The OMPL path length objective.
            Only defined if the method is OMPL path length.
        reference (ci.HelperLatLon): The XY origin when converting from latlon to XY.
            Only defined if the method is latlon.
    """

    def __init__(
        self,
        space_information,
        reference=ci.HelperLatLon(latitude=0.0, longitude=0.0),
    ):
        super().__init__(space_information)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the distance between two points

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            ob.Cost: The distance between two points object

        Raises:
            ValueError: If the distance method is not supported
        """
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        distance = DistanceObjective.get_euclidean_path_length_objective(s1_xy, s2_xy)
        cost = ob.Cost(distance)
        return cost

    @staticmethod
    def get_euclidean_path_length_objective(s1: cs.XY, s2: cs.XY) -> float:
        """Generates the euclidean distance between two points

        Args:
            s1 (cs.XY): The starting point of the local start state
            s2 (cs.XY): The ending point of the local goal state

        Returns:
            float: The euclidean distance between the two points
        """
        return math.hypot(s2.y - s1.y, s2.x - s1.x)


class MinimumTurningObjective(Objective):
    """Generates a minimum turning objective function

    Attributes:
        goal (cs.XY): The goal position of the sailbot
        heading (float): The heading of the sailbot in radians (-pi, pi]
        method (MinimumTurningMethod): The method of the minimum turning objective function
    """

    def __init__(
        self,
        space_information,
        simple_setup,
        heading_degrees: float,
    ):
        super().__init__(space_information)
        self.goal = cs.XY(
            simple_setup.getGoal().getState().getX(), simple_setup.getGoal().getState().getY()
        )
        assert -180 < heading_degrees <= 180
        self.heading = math.radians(heading_degrees)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the turning cost between s1, s2, heading or the goal position

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            ob.Cost: The minimum turning angle in degrees

        Raises:
            ValueError: If the minimum turning method is not supported
        """
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        threshold = math.pi / 9  # 20 degrees around the angle to next waypoint

        # calculate the difference in angle between s1 and s2
        raw_angle_s1_s2 = math.atan2(s2_xy.y - s1_xy.y, s2_xy.x - s1_xy.x)

        # angle between the orientation of the boat at s1 and the location of the s2
        angle_s1_s2 = MinimumTurningObjective.min_turn_angle(raw_angle_s1_s2, s1.getYaw())

        # now we need to ensure that the s2's orientation isn't horrendous given s2_xy
        if MinimumTurningObjective.min_turn_angle(s2.getYaw(), raw_angle_s1_s2) > threshold:
            # the orientation of the boat doesn't make sense even after accounting for drift
            # nuke the cost
            return ob.Cost(3000)
        else:
            return ob.Cost(angle_s1_s2)

    @staticmethod
    def min_turn_angle(angle1: float, angle2: float) -> float:
        """Calculates the minimum turning angle between two angles

        Args:
            angle1 (float): The first angle in radians
            angle2 (float): The second angle in radians
                Must be bounded within 2pi radians of `angle1`

        Returns:
            float: The minimum turning angle between the two angles in radians
        """
        # Calculate the uncorrected turn size [0, 2pi]
        turn_size_bias = math.fabs(angle1 - angle2)

        # Correct the angle in between [0, pi]
        if turn_size_bias > math.pi:
            turn_size_unbias = 2 * math.pi - turn_size_bias
        else:
            turn_size_unbias = turn_size_bias

        return math.fabs(turn_size_unbias)


class WindObjective(Objective):
    """Generates a wind objective function

    Attributes:
        wind_direction (float): The direction of the wind in radians (-pi, pi]
    """

    def __init__(
        self,
        space_information,
        wind_direction_degrees: float,
        wind_speed: float,
        heading_degrees: float,
        speed: float,
    ):
        super().__init__(space_information)
        assert -180 < wind_direction_degrees <= 180
        self.wind_direction, _ = get_true_wind(
            wind_direction_degrees, wind_speed, heading_degrees, speed
        )

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the cost associated with the upwind and downwind directions of the boat in
        relation to the wind.

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            ob.Cost: The cost of going upwind or downwind
        """
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        return ob.Cost(WindObjective.wind_direction_cost(s1_xy, s2_xy, self.wind_direction))

    @staticmethod
    def wind_direction_cost(s1: cs.XY, s2: cs.XY, wind_direction: float) -> float:
        """Punishes the boat for going up/downwind.

        Args:
            s1 (cs.XY): The starting point of the local start state
            s2 (cs.XY): The ending point of the local goal state
            wind_direction (float): The direction of the wind in radians (-pi, pi]

        Returns:
            float: The cost of going upwind or downwind
        """
        distance = math.hypot(s2.y - s1.y, s2.x - s1.x)
        boat_direction_radians = math.atan2(s2.x - s1.x, s2.y - s1.y)
        assert -math.pi <= boat_direction_radians <= math.pi

        if WindObjective.is_upwind(wind_direction, boat_direction_radians):
            return UPWIND_MULTIPLIER * distance
        elif WindObjective.is_downwind(wind_direction, boat_direction_radians):
            return DOWNWIND_MULTIPLIER * distance
        else:
            return 0.0

    @staticmethod
    def is_upwind(wind_direction: float, boat_direction: float) -> bool:
        """Determines whether the boat is upwind or not and its associated cost

        Args:
            wind_direction (float): The true wind direction (radians). (-pi, pi]
            boat_direction (float): The direction of the boat (radians). [-pi, pi]

        Returns:
            bool: The cost associated with the upwind direction
        """
        theta_min = wind_direction - HIGHEST_UPWIND_ANGLE_RADIANS
        theta_max = wind_direction + HIGHEST_UPWIND_ANGLE_RADIANS

        return WindObjective.is_angle_between(theta_min, boat_direction, theta_max)

    @staticmethod
    def is_downwind(wind_direction: float, boat_direction: float) -> bool:
        """Generates the cost associated with the downwind direction

        Args:
            wind_direction (float): The true wind direction (radians). (-pi, pi]
            boat_direction_radians (float)): The direction of the boat (radians). [-pi, pi]

        Returns:
            bool: The cost associated with the downwind direction
        """
        downwind_wind_direction = (wind_direction + math.pi) % (2 * math.pi)

        theta_min = downwind_wind_direction - LOWEST_DOWNWIND_ANGLE_RADIANS

        theta_max = downwind_wind_direction + LOWEST_DOWNWIND_ANGLE_RADIANS

        return WindObjective.is_angle_between(theta_min, boat_direction, theta_max)

    @staticmethod
    def is_angle_between(first_angle: float, middle_angle: float, second_angle: float) -> bool:
        """Determines whether an angle is between two other angles

        Args:
            first_angle (float): The first bounding angle in radians
            middle_angle (float): The angle in question in radians
            second_angle (float): The second bounding angle in radians

        Returns:
            bool: True when `middle_angle` is not in the reflex angle of
                `first_angle` and `second_angle`, false otherwise.
        """
        # Bound the angles to [0, 2pi)
        first_angle = first_angle % (2 * math.pi)
        middle_angle = middle_angle % (2 * math.pi)
        second_angle = second_angle % (2 * math.pi)

        if first_angle <= second_angle:
            if second_angle - math.pi == first_angle:
                # Assume all angles are between first and second
                return middle_angle != first_angle and middle_angle != second_angle
            elif second_angle - math.pi < first_angle:
                return middle_angle > first_angle and middle_angle < second_angle
            else:
                return middle_angle < first_angle or middle_angle > second_angle
        else:
            return WindObjective.is_angle_between(second_angle, middle_angle, first_angle)


class SpeedObjective(Objective):
    """Generates a speed objective function

    Attributes:
        wind_direction (float): The direction of the wind in radians (-pi, pi]
        wind_speed (float): The speed of the wind in m/s
    """

    def __init__(
        self,
        space_information,
        heading_direction: float,
        wind_direction: float,
        wind_speed: float,
    ):
        super().__init__(space_information)
        assert -180 < wind_direction <= 180
        self.wind_direction = math.radians(wind_direction)

        assert -180 < heading_direction <= 180
        self.heading_direction = math.radians(heading_direction)

        self.wind_speed = wind_speed

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the cost associated with the speed of the boat.

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            ob.Cost: The cost of going upwind or downwind
        """

        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())

        sailbot_speed = self.get_sailbot_speed(
            self.heading_direction, self.wind_direction, self.wind_speed
        )

        if sailbot_speed == 0:
            return ob.Cost(10000)

        distance = DistanceObjective.get_euclidean_path_length_objective(s1_xy, s2_xy)
        time = distance / sailbot_speed

        cost = ob.Cost(time)

        return cost

    @staticmethod
    def get_sailbot_speed(heading: float, wind_direction: float, wind_speed: float) -> float:
        # Get the sailing angle: [0, 180]
        sailing_angle = abs(heading - wind_direction)
        sailing_angle = min(sailing_angle, 360 - sailing_angle)

        # Find the nearest windspeed values above and below the true windspeed
        lower_windspeed_index = max([i for i, ws in enumerate(WINDSPEEDS) if ws <= wind_speed])
        upper_windspeed_index = (
            lower_windspeed_index + 1
            if lower_windspeed_index < len(WINDSPEEDS) - 1
            else lower_windspeed_index
        )

        # Find the nearest angle values above and below the sailing angle
        lower_angle_index = max([i for i, ang in enumerate(ANGLES) if ang <= sailing_angle])
        upper_angle_index = (
            lower_angle_index + 1 if lower_angle_index < len(ANGLES) - 1 else lower_angle_index
        )

        # Find the maximum angle and maximum windspeed based on the actual data in the table
        max_angle = max(ANGLES)
        max_windspeed = max(WINDSPEEDS)

        # Handle the case of maximum angle (use the dynamic max_angle)
        if upper_angle_index == len(ANGLES) - 1:
            lower_angle_index = ANGLES.index(max_angle) - 1
            upper_angle_index = ANGLES.index(max_angle)

        # Handle the case of the maximum windspeed (use the dynamic max_windspeed)
        if upper_windspeed_index == len(WINDSPEEDS) - 1:
            lower_windspeed_index = WINDSPEEDS.index(max_windspeed) - 1
            upper_windspeed_index = WINDSPEEDS.index(max_windspeed)

        # Perform linear interpolation
        lower_windspeed = WINDSPEEDS[lower_windspeed_index]
        upper_windspeed = WINDSPEEDS[upper_windspeed_index]
        lower_angle = ANGLES[lower_angle_index]
        upper_angle = ANGLES[upper_angle_index]

        boat_speed_lower = BOATSPEEDS[lower_windspeed_index][lower_angle_index]
        boat_speed_upper = BOATSPEEDS[upper_windspeed_index][lower_angle_index]

        interpolated_1 = boat_speed_lower + (wind_speed - lower_windspeed) * (
            boat_speed_upper - boat_speed_lower
        ) / (upper_windspeed - lower_windspeed)

        boat_speed_lower = BOATSPEEDS[lower_windspeed_index][upper_angle_index]
        boat_speed_upper = BOATSPEEDS[upper_windspeed_index][upper_angle_index]

        interpolated_2 = boat_speed_lower + (wind_speed - lower_windspeed) * (
            boat_speed_upper - boat_speed_lower
        ) / (upper_windspeed - lower_windspeed)

        interpolated_value = interpolated_1 + (sailing_angle - lower_angle) * (
            interpolated_2 - interpolated_1
        ) / (upper_angle - lower_angle)

        return interpolated_value


def get_sailing_objective(
    space_information,
    simple_setup,
    heading_degrees: float,
    speed: float,
    wind_direction_degrees: float,
    wind_speed: float,
) -> ob.OptimizationObjective:
    objective = ob.MultiOptimizationObjective(si=space_information)
    objective.addObjective(
        objective=DistanceObjective(space_information),
        weight=1.0,
    )
    objective.addObjective(
        objective=MinimumTurningObjective(space_information, simple_setup, heading_degrees),
        weight=10.0,
    )
    objective.addObjective(
        objective=WindObjective(
            space_information, wind_direction_degrees, wind_speed, heading_degrees, speed
        ),
        weight=5.0,
    )
    objective.addObjective(
        objective=SpeedObjective(
            space_information,
            heading_degrees,
            wind_direction_degrees,
            wind_speed,
        ),
        weight=0,
    )

    return objective
