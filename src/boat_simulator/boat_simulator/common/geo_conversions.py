"""Geographic helpers for the boat simulator local coordinate frame."""

import math

import numpy as np
from numpy.typing import ArrayLike
from pyproj import Geod

import boat_simulator.common.constants as Constants


GEODESIC = Geod(ellps="WGS84")


def local_position_to_gps_lat_lon(
    local_position_m: ArrayLike,
    origin_latitude: float = Constants.SIM_GPS_ORIGIN_LATITUDE,
    origin_longitude: float = Constants.SIM_GPS_ORIGIN_LONGITUDE,
) -> tuple[float, float]:
    """Convert simulator-local ENU meter offsets into decimal-degree latitude/longitude.

    The simulator's global position uses a local East-North-Up frame in meters:
    `x` is east, `y` is north, and `z` is ignored for GPS. This helper converts the
    horizontal offset into a geodesic distance and bearing from the configured simulator
    GPS origin.

    Args:
        local_position_m (ArrayLike): Simulator-local position in meters. The first value is
            east, the second value is north, and any vertical value is ignored.
        origin_latitude (float): GPS origin latitude in decimal degrees.
        origin_longitude (float): GPS origin longitude in decimal degrees.

    Returns:
        tuple[float, float]: Latitude and longitude in decimal degrees.

    Raises:
        ValueError: If the local position or GPS origin is invalid.
    """
    local_position = _validate_local_position(local_position_m)
    _validate_origin(origin_latitude, origin_longitude)

    east_m = float(local_position[0])
    north_m = float(local_position[1])
    distance_m = math.hypot(east_m, north_m)
    if distance_m == 0.0:
        return float(origin_latitude), float(origin_longitude)

    bearing_deg = math.degrees(math.atan2(east_m, north_m))
    longitude, latitude, _ = GEODESIC.fwd(
        origin_longitude,
        origin_latitude,
        bearing_deg,
        distance_m,
    )

    if not np.isfinite([latitude, longitude]).all():
        raise ValueError("Converted latitude/longitude must be finite")

    return float(latitude), float(longitude)


def _validate_local_position(local_position_m: ArrayLike) -> np.ndarray:
    """Validate simulator-local position input.

    Args:
        local_position_m (ArrayLike): Simulator-local position in meters. Must be a 1D numeric
            array-like value with at least east and north values.

    Returns:
        np.ndarray: The local position as a float array.

    Raises:
        ValueError: If the local position is non-numeric, non-finite, not 1D, or too short.
    """
    try:
        local_position = np.asarray(local_position_m, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("Local position must contain numeric meter offsets") from exc

    if local_position.ndim != 1 or local_position.size < 2:
        raise ValueError("Local position must be a 1D array with at least x and y values")

    if not np.isfinite(local_position[:2]).all():
        raise ValueError("Local position x/y values must be finite")

    return local_position


def _validate_origin(origin_latitude: float, origin_longitude: float) -> None:
    """Validate GPS origin latitude and longitude.

    Args:
        origin_latitude (float): GPS origin latitude in decimal degrees.
        origin_longitude (float): GPS origin longitude in decimal degrees.

    Raises:
        ValueError: If either coordinate is non-finite or outside the decimal-degree GPS bounds.
    """
    if not np.isfinite([origin_latitude, origin_longitude]).all():
        raise ValueError("GPS origin latitude/longitude must be finite")

    if not -90.0 <= origin_latitude <= 90.0:
        raise ValueError("GPS origin latitude must be in decimal degrees within [-90, 90]")

    if not -180.0 <= origin_longitude <= 180.0:
        raise ValueError("GPS origin longitude must be in decimal degrees within [-180, 180]")
