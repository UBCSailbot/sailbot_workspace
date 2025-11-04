"""This file has the helpers related to coordinate systems concerned with wind conversions.
"""

import math
import local_pathfinding.coord_systems as cs
from typing import Tuple

FLOATING_POINT_ERROR_THRESHOLD = 1e-9
ZERO_VECTOR_CONSTANT = 0.0


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


def global_to_boat_coordinate(boat_heading: float, global_wind_direction: float):
    """
    Convert a global-frame wind bearing to a boat-frame wind direction.
    Args:
        boat_heading (float): Boat heading in degrees (global frame), in (-180, 180].
        global_wind_direction (float): Wind bearing in global frame (degrees), in (-180, 180].
    Returns:
        float: Wind direction in boat coordinates (degrees) within (-180, 180].
    """
    return cs.bound_to_180(global_wind_direction - boat_heading + 180.0)


def get_true_wind(
    aw_direction: float,
    aw_speed: float,
    boat_heading: float,
    boat_speed: float,
) -> tuple[float, float]:
    """Compute the true wind vector from apparent wind and boat motion.

    Args:
        aw_direction (float): Apparent wind direction in degrees (-180, 180]. This is the wind
            as measured relative to the boat (sensor reading).
        aw_speed (float): Apparent wind speed (same units as boat_speed), e.g., km/h.
        boat_heading (float): Boat heading in degrees (-180, 180].
        boat_speed (float): Boat speed over ground (same units as aw_speed), e.g., km/h.
        NOTE: All the angles are with respect to global true bearing. It is the
        responsibility of the caller to ensure this. Particularly, the apparent wind read by the
        sensor is in boat coordinates

    Returns:
        tuple[float, float]: (tw_angle, tw_magnitude)
            - tw_angle: true wind direction in radians within (-pi, pi].
            - tw_magnitude: true wind speed
        If the resulting vector magnitude is effectively zero (<= FLOATING_POINT_ERROR_THRESHOLD),
        returns (0.0, 0.0). NOTE: The caller is responsible for handling this case, otherwise the
        calculations will break down.

    Notes:
        The function computes vector components in an east/north frame, subtracting the boat
        motion from the apparent wind to obtain the true wind.
    """
    wind_radians = math.radians(aw_direction)

    # boat wind is in the direction of the boat heading (reverse of boat heading)
    boat_wind_radians = math.radians(cs.bound_to_180(boat_heading + 180))
    apparent_wind_east = aw_speed * math.sin(wind_radians)
    apparent_wind_north = aw_speed * math.cos(wind_radians)

    boat_wind_east = boat_speed * math.sin(boat_wind_radians)
    boat_wind_north = boat_speed * math.cos(boat_wind_radians)

    true_east = apparent_wind_east - boat_wind_east
    true_north = apparent_wind_north - boat_wind_north

    magnitude = math.hypot(true_east, true_north)
    angle = math.atan2(true_east, true_north)

    if magnitude > FLOATING_POINT_ERROR_THRESHOLD:
        return angle, magnitude
    return ZERO_VECTOR_CONSTANT, 0.0


def get_apparent_wind(tw_direction, tw_speed, boat_heading, boat_speed) -> tuple[float, float]:
    """Compute the apparent wind vector from true wind and boat motion.

    Args:
        tw_direction (float): True wind direction in radians (measured as atan2(east, north) style)
            or degrees if caller uses degrees consistently. (This routine converts using math.radians # noqa
            in the original code; keep inputs consistent with usage.)
        tw_speed (float): True wind speed (same units as boat_speed), e.g., km/h.
        boat_heading (float): Boat heading in degrees (-180, 180].
        boat_speed (float): Boat speed over ground (same units as tw_speed), e.g., km/h.
        NOTE: All the angles are with respect to global true bearing. It is the
        responsibility of the caller to ensure this.

    Returns:
        tuple[float, float]: (aw_angle, aw_magnitude)
            - aw_angle: apparent wind direction in radians within (-pi, pi]
            - aw_magnitude: apparent wind speed (same units as inputs).
        If the resulting vector magnitude is effectively zero (<= FLOATING_POINT_ERROR_THRESHOLD),
        returns (0.0, 0.0). NOTE: The caller is responsible for handling this case, otherwise the
        calculations will break down.
    """
    tw_direction = math.radians(tw_direction)

    bw_speed = boat_speed
    bw_direction = math.radians(cs.bound_to_180(boat_heading + 180.0))
    # print(math.degrees(bw_direction))
    boat_wind_east = bw_speed * math.sin(bw_direction)
    boat_wind_north = bw_speed * math.cos(bw_direction)

    tw_speed_east = tw_speed * math.sin(tw_direction)
    tw_speed_north = tw_speed * math.cos(tw_direction)

    aw_speed_east = tw_speed_east + boat_wind_east
    aw_speed_north = tw_speed_north + boat_wind_north

    magnitude = math.hypot(aw_speed_east, aw_speed_north)
    angle = math.atan2(aw_speed_east, aw_speed_north)

    if magnitude > FLOATING_POINT_ERROR_THRESHOLD:
        return angle, magnitude
    return ZERO_VECTOR_CONSTANT, 0.0
