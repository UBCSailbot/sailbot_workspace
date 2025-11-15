"""Functions and classes for converting between coordinate systems."""

import math
from typing import List, NamedTuple

import custom_interfaces.msg as ci
from pyproj import Geod
from shapely.geometry import Point, Polygon

GEODESIC = Geod(ellps="WGS84")
PI = math.pi


class XY(NamedTuple):
    """2D Cartesian coordinate representation.

    Attributes:
        x (float): X coordinate.
        y (float): Y coordinate.
    """

    x: float
    y: float


def cartesian_to_true_bearing(cartesian_angle: float, rad: bool = False) -> float:
    """Convert a cartesian angle to the equivalent true bearing.

    Args:
        cartesian_angle (float): Angle where 0 is east and values increase counter-clockwise.
        rad (bool): If set to true cartesian_angle is assumed to be in radians, otherwise
                    cartesian_angle is assumed to be in degrees by default.

    Returns:
        float: Angle where 0 is north and values increase clockwise. If rad is set to True then the
               returned angle is in radians, otherwise it is in degrees by default.
    """
    if rad:
        return ((PI / 2) - cartesian_angle + (2 * PI)) % (2 * PI)
    return (90 - cartesian_angle + 360) % 360


def true_bearing_to_plotly_cartesian(true_bearing_degrees: float) -> float:
    """Convert a true bearing angle to the equivalent cartesian angle .

    Args:
        true_bearing_degrees (float): Angle where 0 is true north. Range: -180 < heading <= 180.
        Increases in the clockwise direction till 180 degrees.
        Decreases in the counter-clockwise direction till -180 (exclusive)
    Returns:
        float:  Angle where 0 is north and values increases clockwise.
    """
    assert -180 < true_bearing_degrees <= 180

    plotly_cartesian = true_bearing_degrees
    if -180 < true_bearing_degrees < 0:
        plotly_cartesian += 360.0
    return plotly_cartesian


def get_path_segment_true_bearing(s1: XY, s2: XY, rad: bool = False):
    """Returns the true bearing of a straight path from s1 to s2.

    Args:
        s1 (SE2StateInternal): The start of the path segment
        s2 (SE2StateInternal): The end of the path segment
        rad (bool): If set to true the returned true bearing is in radians, otherwise the returned
                    true bearing is in degrees by default

    Returns:
        float: Angle where 0 is north and values increase clockwise. If rad is set to True then the
               returned angle is in radians, otherwise it is in degrees by default.
               The returned angle is always within the range (-180, 180] by default or (-pi, pi]
               radians if rad = True
    """
    segment_cartesian_angle_radians = math.atan2(s2.y - s1.y, s2.x - s1.x)
    segment_true_bearing_radians = cartesian_to_true_bearing(
        segment_cartesian_angle_radians, rad=True
    )

    if rad:
        return bound_to_pi(segment_true_bearing_radians)
    return bound_to_180(math.degrees(segment_true_bearing_radians))


def true_bearing_to_xy_vector(true_bearing_deg: float, speed_knots: float) -> XY:
    """
        Convert a true-bearing direction and scalar speed to east/north Cartesian components.

    Args:
        true_bearing_deg (float): Direction of the vector in **global true-bearing**
        coordinates, measured clockwise from north (0° = north, 90° = east).
        Range: (-180, 180] or [0, 360) depending on context.
        speed_knots (float): Magnitude of the vector, e.g., wind or boat speed (knots).

    Returns:
        cs.XY: Decomposed vector components in the global (east, north) frame:
            - x → east component
            - y → north component

    Notes:
        Converts polar coordinates in true-bearing convention to a 2D Cartesian vector.
        Positive x corresponds to eastward motion; positive y corresponds to northward motion.
        This helper is used to express directional quantities (e.g., apparent wind, true wind)
        in the global reference frame.
    """
    r = math.radians(true_bearing_deg)
    return XY(x=speed_knots * math.sin(r), y=speed_knots * math.cos(r))


def angle_to_xy_vector(angle_rad: float, speed_knots: float) -> XY:
    """
    Convert a polar vector (angle in radians, speed_knots) to east/north Cartesian components.

    Args:
        angle_rad (float): Direction angle in radians following the **atan2(east, north)**
        convention used throughout wind_coord_systems.py. Range: (-π, π], where 0 rad = north,
        +π/2 = east.
        speed_knots (float): Vector speed_knots (e.g., true wind speed, in knots).

    Returns:
        cs.XY: Decomposed vector components in the global (east, north) frame:
            - x → east component
            - y → north component

    Notes:
        This helper converts a polar representation (angle, speed_knots) into its Cartesian
        form. It is primarily used when wind_coord_systems.py returns a direction in radians
        from atan2(). Maintains consistency with the east/north coordinate system used across
        the path visualizer.
    """
    return XY(x=speed_knots * math.sin(angle_rad), y=speed_knots * math.cos(angle_rad))


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
    if math.isclose(angle, -180.0):
        return 180.0
    return angle


def bound_to_pi(angle_radians: float) -> float:
    """Normalize an angle to the range (-pi, pi].

    Args:
        angle_radians (float): Angle in radians to be normalized.

    Returns:
        float: The normalized angle in radians within (-pi, pi].
    """
    angle = ((angle_radians + PI) % (2 * PI)) - PI
    if math.isclose(angle, -PI):
        return PI
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
