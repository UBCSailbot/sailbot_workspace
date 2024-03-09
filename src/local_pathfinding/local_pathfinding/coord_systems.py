"""Functions and classes for converting between coordinate systems."""

import math
from typing import NamedTuple

from custom_interfaces.msg import HelperLatLon
from pyproj import Geod

GEODESIC = Geod(ellps="WGS84")


class XY(NamedTuple):
    """2D Cartesian coordinate representation.

    Attributes:
        x (float): X coordinate.
        y (float): Y coordinate.
    """

    x: float
    y: float


def cartesian_to_true_bearing(cartesian: float) -> float:
    """Convert a cartesian angle to the equivalent true bearing.

    Args:
        cartesian (float): Angle where 0 is east and values increase counter-clockwise.

    Returns:
        float: Angle where 0 is north and values increase clockwise.
    """
    return (90 - cartesian + 360) % 360


def meters_to_km(meters: float) -> float:
    return meters / 1000


def km_to_meters(km: float) -> float:
    return km * 1000


def latlon_to_xy(reference: HelperLatLon, latlon: HelperLatLon) -> XY:
    """Convert a geographical coordinate to a 2D Cartesian coordinate given a reference point.

    Args:
        reference (HelperLatLon): Origin of the Cartesian coordinate system.
        latlon (HelperLatLon): Coordinate to be converted to the Cartesian coordinate system.

    Returns:
        XY: The x and y components in km.
    """
    forward_azimuth_deg, _, distance_m = GEODESIC.inv(
        reference.longitude, reference.latitude, latlon.longitude, latlon.latitude
    )
    true_bearing = math.radians(forward_azimuth_deg)
    distance = meters_to_km(distance_m)

    return XY(
        x=distance * math.sin(true_bearing),
        y=distance * math.cos(true_bearing),
    )


def xy_to_latlon(reference: HelperLatLon, xy: XY) -> HelperLatLon:
    """Convert a 2D Cartesian coordinate to a geographical coordinate given a reference point.

    Args:
        reference (HelperLatLon): Coordinate that is the origin of the Cartesian coordinate system.
        xy (XY): Coordinate to be converted to the geographical coordinate system.

    Returns:
        HelperLatLon: The latitude and longitude in degrees.
    """
    true_bearing = math.degrees(math.atan2(xy.x, xy.y))
    distance = km_to_meters(math.hypot(*xy))
    dest_lon, dest_lat, _ = GEODESIC.fwd(
        reference.longitude, reference.latitude, true_bearing, distance
    )

    return HelperLatLon(latitude=dest_lat, longitude=dest_lon)
