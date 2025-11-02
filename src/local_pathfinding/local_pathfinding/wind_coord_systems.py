"""This file has the helpers related to coordinate systems concerned with wind conversions.
"""

import math
import local_pathfinding.coord_systems as cs
from typing import Tuple


def boat_to_global_coordinate(boat_heading: float, wind_direction: float):
    """
    Convert a boat-frame wind direction to a global-frame bearing (degrees) in (-180, 180]
    Args:
        boat_heading (float): Boat heading in degrees (global frame).
        wind_direction (float): Wind direction in boat coordinates (degrees).
    Returns:
        float: Global wind bearing in degrees within [-180, 180].
    Examples:
        >>> boat_to_global_coordinate(0, 0)      # 0 + 0 + 180 -> -180
        180
        >>> boat_to_global_coordinate(170, 30)   # 170 + 30 + 180 = 380 -> 20
        20
    """
    return cs.bound_to_180(boat_heading + wind_direction + 180.0)

# def global_to_boat_coordinate(boat_heading: float, global_wind_direction: float):




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
    boat_wind_radians = math.radians(cs.bound_to_180(heading_degrees + 180))

    apparent_wind_east = apparent_wind_speed * math.sin(wind_radians)
    apparent_wind_north = apparent_wind_speed * math.cos(wind_radians)

    boat_wind_east = boat_speed_over_ground * math.sin(boat_wind_radians)
    boat_wind_north = boat_speed_over_ground * math.cos(boat_wind_radians)

    true_east = apparent_wind_east - boat_wind_east
    true_north = apparent_wind_north - boat_wind_north

    return (math.atan2(true_east, true_north), math.hypot(true_north, true_east))


def get_apparent_wind(self)-> tuple[float, float]:
    tw_speed = self.get_mock_wind_speed().speed
    tw_direction = math.radians(self.get_direction_value())

    bw_speed = self.__boat_speed
    bw_direction = math.radians(cs.bound_to_180(self.__heading + 180.0))

    boat_wind_east = bw_speed * math.sin(bw_direction)
    boat_wind_north = bw_speed * math.cos(bw_direction)
    return 0.0
