"""This file has the helpers related to coordinate systems concerned with wind conversions.
NOTE: Boat coordinate is defined with 0 degrees in the opposite direction of the boat's heading.
The value increases clockwise wrt to the boat coordinate and decreases counter-clockwise. Follows
true bearing conventions.
"""

import math

import local_pathfinding.coord_systems as cs

FLOATING_POINT_ERROR_THRESHOLD = 1e-9
ZERO_VECTOR_CONSTANT = 0.0


def boat_to_global_coordinate(boat_heading: float, wind_direction: float):
    """
    Convert a boat-frame wind direction to a global-frame bearing (degrees) in (-180, 180]
    Args:
        boat_heading (float): Boat heading in degrees (global frame) in (-180, 180].
        wind_direction (float): Wind direction in boat coordinates (-180, 180] (degrees).
    Returns:
        float: Global wind bearing in degrees within (-180, 180].
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
    aw_dir_deg: float,
    aw_speed_kmph: float,
    boat_heading_deg: float,
    boat_speed_kmph: float,
    ret_rad: bool = True,
) -> tuple[float, float]:
    """Compute the true wind vector from apparent wind and boat motion.

    Args:
        aw_dir_deg (float): Apparent wind direction in degrees (-180, 180]. This is the wind
            as measured relative to the boat (sensor reading).
        aw_speed_kmph (float): Apparent wind speed in km/h.
        boat_heading_deg (float): Boat heading in degrees (-180, 180].
        boat_speed_kmph (float): Boat speed over ground in km/h.
        ret_rad (bool): If True, return wind direction in radians. If False, return in degrees.
        NOTE: All the angles are with respect to global true bearing. It is the
        responsibility of the caller to ensure this. Particularly, the apparent wind read by the
        sensor is in boat coordinates

    Returns:
        tuple[float, float]: (tw_dir_rad, tw_speed_kmph)
            - tw_dir_rad: true wind direction in radians within (-pi, pi].
            - tw_speed_kmph: true wind speed in km/h
        If the resulting vector magnitude is effectively zero (<= FLOATING_POINT_ERROR_THRESHOLD),
        returns (0.0, 0.0). NOTE: The caller is responsible for handling this case, otherwise the
        calculations will break down.

    Notes:
        The function computes vector components in an east/north frame, subtracting the boat
        motion from the apparent wind to obtain the true wind.
    """
    aw_dir_rad = math.radians(aw_dir_deg)

    # boat wind is in the direction of the boat heading (reverse of boat heading)
    bw_dir_rad = math.radians(cs.bound_to_180(boat_heading_deg + 180))
    aw_east_kmph = aw_speed_kmph * math.sin(aw_dir_rad)
    aw_north_kmph = aw_speed_kmph * math.cos(aw_dir_rad)

    bw_east_kmph = boat_speed_kmph * math.sin(bw_dir_rad)
    bw_north_kmph = boat_speed_kmph * math.cos(bw_dir_rad)

    tw_east_kmph = aw_east_kmph - bw_east_kmph
    tw_north_kmph = aw_north_kmph - bw_north_kmph

    tw_speed_kmph = math.hypot(tw_east_kmph, tw_north_kmph)
    tw_dir_rad = math.atan2(tw_east_kmph, tw_north_kmph)

    if ret_rad:
        tw_dir = tw_dir_rad
    else:
        tw_dir = math.degrees(tw_dir_rad)

    if tw_speed_kmph > FLOATING_POINT_ERROR_THRESHOLD:
        return tw_dir, tw_speed_kmph
    return ZERO_VECTOR_CONSTANT, 0.0


def get_apparent_wind(
    tw_dir_deg: float,
    tw_speed_kmph: float,
    boat_heading_deg: float,
    boat_speed_kmph: float,
    ret_rad: bool = True,
) -> tuple[float, float]:
    """Compute the apparent wind vector from true wind and boat motion.

    Args:
        tw_dir_deg (float): True wind direction in degrees (-180, 180].
        tw_speed_kmph (float): True wind speed in km/h.
        boat_heading_deg (float): Boat heading in degrees (-180, 180].
        boat_speed_kmph (float): Boat speed over ground in km/h.
        ret_rad (bool): If True, return wind direction in radians. If False, return in degrees.
        NOTE: All the angles are with respect to global true bearing. It is the
        responsibility of the caller to ensure this.

    Returns:
        tuple[float, float]: (aw_dir_rad, aw_speed_kmph)
            - aw_dir_rad: apparent wind direction in radians within (-pi, pi]
            - aw_speed_kmph: apparent wind speed in km/h.
        If the resulting vector magnitude is effectively zero (<= FLOATING_POINT_ERROR_THRESHOLD),
        returns (0.0, 0.0). NOTE: The caller is responsible for handling this case, otherwise the
        calculations will break down.
    """
    tw_dir_rad = math.radians(tw_dir_deg)

    bw_speed_kmph = boat_speed_kmph
    bw_dir_rad = math.radians(cs.bound_to_180(boat_heading_deg + 180.0))
    bw_east_kmph = bw_speed_kmph * math.sin(bw_dir_rad)
    bw_north_kmph = bw_speed_kmph * math.cos(bw_dir_rad)

    tw_east_kmph = tw_speed_kmph * math.sin(tw_dir_rad)
    tw_north_kmph = tw_speed_kmph * math.cos(tw_dir_rad)

    aw_east_kmph = tw_east_kmph + bw_east_kmph
    aw_north_kmph = tw_north_kmph + bw_north_kmph

    aw_speed_kmph = math.hypot(aw_east_kmph, aw_north_kmph)
    aw_dir_rad = math.atan2(aw_east_kmph, aw_north_kmph)

    if ret_rad:
        aw_dir = aw_dir_rad
    else:
        aw_dir = math.degrees(aw_dir_rad)

    if aw_speed_kmph > FLOATING_POINT_ERROR_THRESHOLD:
        return aw_dir, aw_speed_kmph
    return ZERO_VECTOR_CONSTANT, 0.0


def get_true_wind_angle(boat_heading_rad: float, tw_dir_rad: float) -> float:
    """Calculates the true wind angle (TWA) in radians.
    The TWA is the angle between the heading of a vessel and the true wind direction.

    Here are some practical examples of True Wind Angle values:

        0   : Wind directly from the bow (head wind)
        45  : Wind from 45 degrees off the starboard bow (close-hauled)
        -90 : Wind directly from the port side (beam reach)
        135 : Wind from the starboard quarter (broad reach)
        180 : Wind directly from astern (running downwind)

    Args:
        boat_heading_rad (float): _description_
        tw_dir_rad (float): _description_

    Returns:
        float: _description_
    """
    return cs.bound_to_180((tw_dir_rad - boat_heading_rad), rad=True)
