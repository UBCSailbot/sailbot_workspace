"""Functions and classes for converting between coordinate systems."""

import math
from typing import List, NamedTuple

from custom_interfaces.msg import HelperLatLon
from pyproj import Geod
from shapely.geometry import Point, Polygon

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


def latlon_polygon_list_to_xy_polygon_list(
    polygons: List[Polygon], reference: HelperLatLon
) -> List[Polygon]:
    """
    Transforms a list of one or more polygons from the global lat/lon coordinate system to
    the local XY coordinate system.

    Args:
        polygons (List[Polygon]): List of polygons to be transformed.
        reference (HelperLatLon): Lat and lon position of the reference point.

    Returns:
        List[Polygon]: List of transformed polygons.

    Inner Functions:
        _latlon_to_xy_point(point: HelperLatLon) -> Point:
            Converts a latlon point to a 2D Cartesian point.
        _latlons_to_xy_points(poly: Polygon) -> Polygon:
            Applies the _latlon_to_point function to every point of poly
    """

    def _latlon_polygon_to_xy_polygon(poly: Polygon) -> Polygon:
        if poly.is_empty:
            return poly

        return Polygon(list(map(_latlon_point_to_xy_point, poly.exterior.coords)))

    def _latlon_point_to_xy_point(latlon_point: tuple) -> Point:
        return Point(
            *latlon_to_xy(
                reference=reference,
                # points are (lon, lat) in the land dataset
                latlon=HelperLatLon(longitude=latlon_point[0], latitude=latlon_point[1]),
            )
        )

    return list(map(_latlon_polygon_to_xy_polygon, polygons))
