"""Helpers for flow-toward wind directions and coordinate conversions.

Every wind direction points where the air travels. Global directions use true
bearing (0 degrees north, increasing clockwise). The WindSensor boat frame uses
0 degrees opposite the boat heading, so 0 degrees points toward the stern, and
also increases clockwise.
"""

import math
from dataclasses import dataclass

import local_pathfinding.coord_systems as cs

FLOATING_POINT_ERROR_THRESHOLD = 1e-9
ZERO_VECTOR_CONSTANT = 0.0


@dataclass
class Wind:
    """Represents wind speed and direction.

    Attributes:
        speed_kmph (float): Wind speed in kilometers per hour.
        dir_deg (float): Flow-toward wind direction in degrees, bounded to (-180, 180].
    """
    speed_kmph: float
    dir_deg: float


def boat_to_global_coordinate(boat_heading_deg_gc: float, aw_dir_deg_bc: float):
    """
    Convert a boat-frame flow direction to a global flow bearing in (-180, 180].
    Args:
        boat_heading_deg_gc (float): Boat heading in degrees (global frame) in (-180, 180].
        aw_dir_deg_bc (float): Flow-toward wind direction in boat coordinates (-180, 180].
    Returns:
        float: Global flow-toward wind bearing in degrees within (-180, 180].
    Examples:
        >>> boat_to_global_coordinate(0, 0)      # 0 + 0 -> 180
        180
        >>> boat_to_global_coordinate(170, 30)   # 170 + 30 = 380 -> 20
        20
    """
    return cs.bound_to_180(boat_heading_deg_gc + aw_dir_deg_bc + 180)


def global_to_boat_coordinate(boat_heading_deg_gc: float, tw_dir_deg_gc: float):
    """
    Convert a global flow bearing to a boat-frame flow direction.
    Args:
        boat_heading_deg_gc (float): Boat heading in degrees (global frame), in (-180, 180].
        tw_dir_deg_gc (float): Flow-toward wind bearing in global degrees (-180, 180].
    Returns:
        float: Flow-toward wind direction in boat coordinates within (-180, 180].
    Examples:
        >>> global_to_boat_coordinate(0, 0)      # 0 - 0 -> 180
        180
        >>> global_to_boat_coordinate(170, 30)   # 30 - 170 -> 40
        40
    """
    return cs.bound_to_180(tw_dir_deg_gc - boat_heading_deg_gc + 180.0)


def aw_gc_to_tw_gc(
    aw_dir_deg_gc: float,
    aw_speed_kmph: float,
    boat_heading_deg_gc: float,
    boat_speed_kmph: float,
) -> tuple[float, float]:
    """Compute the true wind vector from apparent wind and boat motion.

    Args:
        aw_dir_deg_gc (float): Apparent-wind flow bearing in degrees (-180, 180].
        aw_speed_kmph (float): Apparent wind speed in km/h.
        boat_heading_deg_gc (float): Boat heading in degrees (-180, 180].
        boat_speed_kmph (float): Boat speed over ground in km/h.
        NOTE: All the angles are with respect to global true bearing. It is the
        responsibility of the caller to ensure this.

    Returns:
        tuple[float, float]: (tw_dir_gc, tw_speed_kmph)
            - tw_dir_gc: true-wind flow bearing in degrees within (-180, 180].
            - tw_speed_kmph: true wind speed in km/h
        If the resulting vector magnitude is effectively zero (<= FLOATING_POINT_ERROR_THRESHOLD),
        returns (0.0, 0.0). NOTE: The caller is responsible for handling this case, otherwise the
        calculations will break down.

    Notes:
        The function computes vector components in an east/north frame, subtracting the boat
        wind from the apparent wind to obtain the true wind.
    """
    aw_dir_rad_gc = math.radians(aw_dir_deg_gc)

    # boat wind is reverse of boat heading
    bw_dir_rad_gc = math.radians(cs.bound_to_180(boat_heading_deg_gc + 180))
    aw_east_kmph = aw_speed_kmph * math.sin(aw_dir_rad_gc)
    aw_north_kmph = aw_speed_kmph * math.cos(aw_dir_rad_gc)

    bw_east_kmph = boat_speed_kmph * math.sin(bw_dir_rad_gc)
    bw_north_kmph = boat_speed_kmph * math.cos(bw_dir_rad_gc)

    tw_east_kmph = aw_east_kmph - bw_east_kmph
    tw_north_kmph = aw_north_kmph - bw_north_kmph

    tw_speed_kmph = math.hypot(tw_east_kmph, tw_north_kmph)
    tw_dir_rad_gc = math.atan2(tw_east_kmph, tw_north_kmph)
    tw_dir_gc = math.degrees(tw_dir_rad_gc)

    if tw_speed_kmph > FLOATING_POINT_ERROR_THRESHOLD:
        return tw_dir_gc, tw_speed_kmph
    return ZERO_VECTOR_CONSTANT, 0.0


def tw_gc_to_aw_gc(
    tw_dir_deg_gc: float,
    tw_speed_kmph: float,
    boat_heading_deg_gc: float,
    boat_speed_kmph: float,
) -> tuple[float, float]:
    """Compute the apparent wind vector from true wind and boat motion.

    Args:
        tw_dir_deg_gc (float): True-wind flow bearing in degrees (-180, 180].
        tw_speed_kmph (float): True wind speed in km/h.
        boat_heading_deg_gc (float): Boat heading in degrees (-180, 180].
        boat_speed_kmph (float): Boat speed over ground in km/h.
        NOTE: All the angles are with respect to global true bearing. It is the
        responsibility of the caller to ensure this.

    Returns:
        tuple[float, float]: (aw_dir_gc, aw_speed_kmph)
            - aw_dir_gc: apparent-wind flow bearing in degrees within (-180, 180].
            - aw_speed_kmph: apparent wind speed in km/h.
        If the resulting vector magnitude is effectively zero (<= FLOATING_POINT_ERROR_THRESHOLD),
        returns (0.0, 0.0). NOTE: The caller is responsible for handling this case, otherwise the
        calculations will break down.

    Notes:
        The function computes vector components in an east/north frame, summing the boat
        wind with the true wind to obtain the apparent wind.
    """
    tw_dir_rad_gc = math.radians(tw_dir_deg_gc)

    bw_speed_kmph = boat_speed_kmph
    bw_dir_rad_gc = math.radians(cs.bound_to_180(boat_heading_deg_gc + 180.0))
    bw_east_kmph = bw_speed_kmph * math.sin(bw_dir_rad_gc)
    bw_north_kmph = bw_speed_kmph * math.cos(bw_dir_rad_gc)

    tw_east_kmph = tw_speed_kmph * math.sin(tw_dir_rad_gc)
    tw_north_kmph = tw_speed_kmph * math.cos(tw_dir_rad_gc)

    aw_east_kmph = tw_east_kmph + bw_east_kmph
    aw_north_kmph = tw_north_kmph + bw_north_kmph

    aw_speed_kmph = math.hypot(aw_east_kmph, aw_north_kmph)
    aw_dir_rad_gc = math.atan2(aw_east_kmph, aw_north_kmph)
    aw_dir_gc = math.degrees(aw_dir_rad_gc)

    if aw_speed_kmph > FLOATING_POINT_ERROR_THRESHOLD:
        return aw_dir_gc, aw_speed_kmph
    return ZERO_VECTOR_CONSTANT, 0.0


def get_true_wind_angle(
    boat_heading_rad_gc: float,
    tw_dir_rad_gc: float,
    ret_rad: bool = True
) -> float:
    """Calculate the signed flow-relative true-wind angle.

    The result is the angle from the vessel heading to the direction the true
    airflow travels, increasing clockwise in true-bearing coordinates.

    Here are some practical examples of True Wind Angle values:

        0   : Airflow travels with the heading, toward the bow (directly downwind)
        45  : Airflow travels 45 degrees clockwise from the heading
        -90 : Airflow travels toward port, perpendicular to the heading
        135 : Airflow travels 135 degrees clockwise from the heading
        180 : Airflow travels against the heading, toward the stern (directly upwind)

    Args:
        boat_heading_rad_gc (float): Boat heading in radians (-pi, pi].
        tw_dir_rad_gc (float): True-wind flow bearing in radians (-pi, pi].
        ret_rad (bool): If True, return the relative angle in radians. Otherwise, degrees.

    Returns:
        float: Signed flow-relative angle in the range (-pi, pi].
    """
    result = cs.bound_to_180((tw_dir_rad_gc - boat_heading_rad_gc), rad=True)
    if ret_rad:
        return result
    return math.degrees(result)
