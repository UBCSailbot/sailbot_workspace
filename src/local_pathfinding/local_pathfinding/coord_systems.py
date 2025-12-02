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


def true_bearing_to_plotly_cartesian(true_bearing: float) -> float:
    """Convert a true bearing angle to the equivalent cartesian angle .

    Args:
        true_bearing (float): Angle where 0 is true north. Range: -180 < heading <= 180.
        Increases in the clockwise direction till 180 degrees.
        Decreases in the counter-clockwise direction till -180 (exclusive)
    Returns:
        float:  Angle where 0 is north and values increases clockwise.
    """
    assert -180 < true_bearing <= 180

    plotly_cartesian = true_bearing
    if -180 < true_bearing < 0:
        plotly_cartesian += 360.0
    return plotly_cartesian


def angle_to_vector_projections(vector_angle_rad: float, vector_magnitude: float) -> XY:
    """Convert a polar vector (angle in radians, speed_knots) to east/north Cartesian components.

    Args:
        vector_angle_rad (float): Direction angle in radians. Range: (-π, π], where 0 rad = north,
            +π/2 = east.
        vector_magnitude (float): Vector speed (e.g., true wind speed in kmph).

    Returns:
        XY: Decomposed vector components in the global (east, north) frame:
            - x → east component
            - y → north component
    """
    # case 1: vector in quadrant I
    if 0 <= vector_angle_rad <= math.pi / 2:
        return XY(
            x=vector_magnitude * math.sin(vector_angle_rad),
            y=vector_magnitude * math.cos(vector_angle_rad),
        )
    # case 2: vector in quadrant IV
    elif math.pi / 2 < vector_angle_rad <= math.pi:
        alpha = vector_angle_rad - (math.pi / 2)  # alpha is with respect to positive x-axis
        return XY(x=vector_magnitude * math.cos(alpha), y=vector_magnitude * -1 * math.sin(alpha))
    # case 3: vector in quadrant II
    elif -math.pi / 2 <= vector_angle_rad < 0:
        alpha = abs(vector_angle_rad)  # vector_angle_rad is negative in quadrant II
        return XY(x=vector_magnitude * -1 * math.sin(alpha), y=vector_magnitude * math.cos(alpha))
    # case 4: vector in quadrant III
    elif -math.pi <= vector_angle_rad < -math.pi / 2:
        # vector_angle_rad is negative in quadrant III and alpha is with respect to negative x-axis
        alpha = abs(vector_angle_rad) - (math.pi / 2)
        return XY(
            x=vector_magnitude * -1 * math.cos(alpha), y=vector_magnitude * -1 * math.sin(alpha)
        )
    return XY(x=0.0, y=0.0)  # place holder for invalid input


def meters_to_km(meters: float) -> float:
    return meters / 1000


def km_to_meters(km: float) -> float:
    return km * 1000


def bound_to_180(angle_degrees: float) -> float:
    """Normalize an angle to the range (-180, 180].

    Args:
        angle_degrees (float): Angle in degrees to be normalized.

    Returns:
        float: The normalized angle in degrees within (-180, 180].
    """
    angle = ((angle_degrees + 180) % 360) - 180
    if angle == -180.0:
        return 180.0
    return angle


def calculate_heading_diff(heading1: float, heading2: float):
    """
    calculates the difference in heading between any 2 headings

    Args:
        heading1: the first heading
        heading2: the second heading

    Returns:
        The absolute heading difference with minimum being 0.0 and maximum being 180.0
    """
    return abs(bound_to_180(heading2 - heading1))


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
