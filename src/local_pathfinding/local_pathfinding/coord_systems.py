"""Functions and classes for converting between coordinate systems."""

import math
from typing import List, NamedTuple

import custom_interfaces.msg as ci
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


def bound_to_180(angle_degrees: float) -> float:
    return ((angle_degrees + 180) % 360) - 180


def latlon_to_xy(reference: ci.HelperLatLon, latlon: ci.HelperLatLon) -> XY:
    """Convert a geographical coordinate to a 2D Cartesian coordinate given a reference point.

    Args:
        reference (ci.HelperLatLon): Origin of the Cartesian coordinate system.
        latlon (ci.HelperLatLon): Coordinate to be converted to the Cartesian coordinate system.

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


def xy_to_latlon(reference: ci.HelperLatLon, xy: XY) -> ci.HelperLatLon:
    """Convert a 2D Cartesian coordinate to a geographical coordinate given a reference point.

    Args:
        reference (ci.HelperLatLon): Coordinate that is the origin of the Cartesian coordinate
                                     system.
        xy (XY): Coordinate to be converted to the geographical coordinate system.

    Returns:
        ci.HelperLatLon: The latitude and longitude in degrees.
    """
    true_bearing = math.degrees(math.atan2(xy.x, xy.y))
    distance = km_to_meters(math.hypot(*xy))
    dest_lon, dest_lat, _ = GEODESIC.fwd(
        reference.longitude, reference.latitude, true_bearing, distance
    )

    return ci.HelperLatLon(latitude=dest_lat, longitude=dest_lon)


def xy_polygon_to_latlon_polygon(reference: ci.HelperLatLon, poly: Polygon):
    """
    Transforms a polygon in XY coordinates to a rectangular polygon in lat lon coordinates.
    """
    if poly.is_empty:
        return poly

    def _xy_point_to_latlon_point(xy_point: XY) -> Point:
        latlon = xy_to_latlon(reference=reference, xy=xy_point)
        return Point(latlon.longitude, latlon.latitude)

    return Polygon(
        list(
            map(
                _xy_point_to_latlon_point,
                [XY(x=point[0], y=point[1]) for point in poly.exterior.coords],
            )
        )
    )


def latlon_polygon_list_to_xy_polygon_list(
    polygons: List[Polygon], reference: ci.HelperLatLon
) -> List[Polygon]:
    """
    Transforms a list of one or more polygons from the global lat/lon coordinate system to
    the local XY coordinate system.

    Args:
        polygons (List[Polygon]): List of polygons to be transformed.
        reference (ci.HelperLatLon): Lat and lon position of the reference point.

    Returns:
        List[Polygon]: List of transformed polygons.

    Inner Functions:
        _latlon_to_xy_point(point: ci.HelperLatLon) -> Point:
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
                latlon=ci.HelperLatLon(longitude=latlon_point[0], latitude=latlon_point[1]),
            )
        )

    return list(map(_latlon_polygon_to_xy_polygon, polygons))


def latlon_list_to_xy_list(reference_latlon, lat_lon_list: List[ci.HelperLatLon]) -> List[XY]:
    """Converts a list of lat/lon coordinates to x/y coordinates."""
    return [latlon_to_xy(reference=reference_latlon, latlon=pos) for pos in lat_lon_list]
