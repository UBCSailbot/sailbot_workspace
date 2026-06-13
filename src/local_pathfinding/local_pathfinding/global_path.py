"""The Global Path Module, which retrieves the global path from a constant array and
sends it to NET via POST request.

The main function accepts one CLI argument:
    --interval (float, Optional): The desired path interval length in km.
"""

import argparse
import json
import time
from datetime import datetime
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

import numpy as np

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs

GPS_URL = "http://localhost:3005/api/gps"
PATH_URL = "http://localhost:8081/global-path"
PERIOD = 5  # seconds

# Global path defined as a list of (latitude, longitude) tuples
GLOBAL_PATH_WAYPOINTS = [
    (49.276303, -123.199690),
    (49.281413, -123.193666),
    (49.281634, -123.185532),
    (49.281736, -123.173314),
    (49.289782, -123.171669),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interval", help="Desired path interval length.", type=float)
    args = parser.parse_args()

    path = get_path()
    print("Retrieved path from constant array:", path_to_dict(path))

    # Main service loop
    while True:
        time.sleep(PERIOD)

        pos = get_pos()

        if pos is None:
            print(f"Failed to retrieve position from {GPS_URL}")
            continue

        position_delta = cs.meters_to_km(
            cs.GEODESIC.inv(
                lats1=pos.latitude,
                lons1=pos.longitude,
                lats2=path.waypoints[0].latitude,
                lons2=path.waypoints[0].longitude,
            )[2]
        )

        if (args.interval is None) or position_delta <= args.interval:
            continue

        if args.interval is not None:
            path = interpolate_path(
                path=path,
                pos=pos,
                interval_spacing=args.interval,
            )

        if post_path(path):
            print("Global path successfully updated.")
            print(f"position was {pos}")
        else:
            continue


def get_path() -> ci.Path:
    """Returns the global path from the GLOBAL_PATH_WAYPOINTS constant.

    Returns:
        (ci.Path): The global path.
    """
    path = ci.Path()
    for lat, lon in GLOBAL_PATH_WAYPOINTS:
        path.waypoints.append(ci.HelperLatLon(latitude=lat, longitude=lon))
    return path


def post_path(path: ci.Path, url: str = PATH_URL) -> bool:
    """Sends the global path to NET via POST request.

    Args:
        path (ci.Path): The global path.
        url (str): URL to post path to. Should always use default value except for tests.

    Returns:
        bool: Whether or not the global path was successfully posted.
    """
    waypoints = [
        {"latitude": float(item.latitude), "longitude": float(item.longitude)}
        for item in path.waypoints
    ]

    # the timestamp format will be <last 2 digits of year>-<month>-<day> <hour>:<minute>:<seconds>
    timestamp = datetime.now().strftime("%y-%m-%d %H:%M:%S")

    data = {"waypoints": waypoints, "timestamp": timestamp}

    json_data = json.dumps(data).encode("utf-8")

    try:
        urlopen(url, json_data)
        return True
    except HTTPError as http_error:
        print(f"HTTP Error: {http_error.code}")
    except URLError as url_error:
        print(f"URL Error: {url_error.reason}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    return False


def get_pos() -> ci.HelperLatLon:
    """Returns the current position of sailbot, retrieved from the an http GET request.

    Returns:
        ci.HelperLatLon: The current position of sailbot
            OR
        None: If the position could not be retrieved.
    """
    try:
        position = json.loads(urlopen(GPS_URL).read())
    except HTTPError as http_error:
        print(f"HTTP Error: {http_error.code}")
        return None  # type: ignore
    except URLError as url_error:
        print(f"URL Error: {url_error.reason}")
        return None  # type: ignore
    except ConnectionResetError as connect_error:
        print(f"Connection Reset Error: {connect_error}")
        return None  # type: ignore
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None  # type: ignore

    if len(position["data"]) == 0:
        print(f"Connection to {GPS_URL} successful. No position data available.")
        return None  # type: ignore

    latitude = position["data"][-1]["latitude"]
    longitude = position["data"][-1]["longitude"]
    pos = ci.HelperLatLon(latitude=latitude, longitude=longitude)

    return pos


def generate_path(
    dest: ci.HelperLatLon,
    interval_spacing: float,
    pos: ci.HelperLatLon,
) -> ci.Path:
    """Returns a path from the current GPS location to the destination point.
    Waypoints are evenly spaced along the path according to the interval_spacing parameter.
    Path does not include pos, but does include dest as the last element.

    Args:
        dest (ci.HelperLatLon): The destination point
        interval_spacing (float): The desired distance between waypoints on the path
        pos (ci.HelperLatLon): The current GPS location

    Returns:
        ci.Path: The generated path
    """
    global_path = ci.Path()

    lat1 = pos.latitude
    lon1 = pos.longitude

    lat2 = dest.latitude
    lon2 = dest.longitude

    distance = cs.meters_to_km(cs.GEODESIC.inv(lats1=lat1, lons1=lon1, lats2=lat2, lons2=lon2)[2])

    # minimum number of waypoints to not exceed interval_spacing
    n = np.floor(distance / interval_spacing)
    n = max(1, n)

    # npts returns a path with neither pos nor dest included
    global_path_tuples = cs.GEODESIC.npts(lon1=lon1, lat1=lat1, lon2=lon2, lat2=lat2, npts=n)

    # npts returns (lon,lat) tuples, its backwards for some reason
    for lon, lat in global_path_tuples:
        global_path.waypoints.append(ci.HelperLatLon(latitude=lat, longitude=lon))

    # append the destination point
    global_path.waypoints.append(ci.HelperLatLon(latitude=lat2, longitude=lon2))

    return global_path


def _interpolate_path(
    global_path: ci.Path,
    interval_spacing: float,
    pos: ci.HelperLatLon,
    path_spacing: list[float],
) -> ci.Path:
    """Interpolates and inserts geodesic subpaths between any waypoints which are spaced too far
       apart. This functions uses the convention that the last point in the global path is the next
       global waypoint for sailbot to sail to.

    Args:
        global_path (ci.Path): The path to interpolate between
        interval_spacing (float): The desired spacing between waypoints
        pos (ci.HelperLatLon): The current GPS location
        path_spacing (list[float]): The distances between pairs of points in global_path

    Returns:
        ci.Path: The interpolated path
    """

    waypoints = global_path.waypoints + [pos]

    i, j = 0, 0
    while i < len(path_spacing):
        if path_spacing[i] > interval_spacing:
            # interpolate a new sub path between the two waypoints
            pos = waypoints[j]
            dest = waypoints[j + 1]

            sub_path = generate_path(
                dest=dest,
                interval_spacing=interval_spacing,
                pos=pos,
            )
            # insert sub path into path
            waypoints[j + 1 : j + 1] = sub_path.waypoints[:-1]
            # shift indices to account for path insertion
            j += len(sub_path.waypoints) - 1

        i += 1
        j += 1
    # remove pos from waypoints again
    waypoints.pop(-1)

    global_path.waypoints = waypoints

    return global_path


def interpolate_path(
    path: ci.Path,
    pos: ci.HelperLatLon,
    interval_spacing: float,
) -> ci.Path:
    """Interpolates path to ensure the interval lengths are less than or equal to the specified
    interval spacing.

    Args:
        path (ci.Path): The global path.
        pos (ci.HelperLatLon): The current position of the vehicle.
        interval_spacing (float): The desired interval spacing.

    Returns:
        ci.Path: The interpolated path.
    """

    # obtain the actual distances between every waypoint in the path
    path_spacing = calculate_interval_spacing(pos, path.waypoints)

    # check if global path is just a destination point
    if len(path.waypoints) < 2:
        path = generate_path(
            dest=path.waypoints[0],
            interval_spacing=interval_spacing,
            pos=pos,
        )
    # Check if any waypoints are too far apart
    elif max(path_spacing) > interval_spacing:
        path = _interpolate_path(
            global_path=path,
            interval_spacing=interval_spacing,
            pos=pos,
            path_spacing=path_spacing,
        )

    return path


def calculate_interval_spacing(
    pos: ci.HelperLatLon, waypoints: list[ci.HelperLatLon]
) -> list[float]:
    """Returns the distances in km between pairs of points in a list of latitudes and
    longitudes, including pos as the last point.

    Args:
        pos (ci.HelperLatLon): The gps position of the boat
        waypoints (list[ci.HelperLatLon]): The list of waypoints

    Returns:
        list[float]: The distances between pairs of points in waypoints [km]
    """
    all_coords = [(waypoint.latitude, waypoint.longitude) for waypoint in waypoints] + [
        (pos.latitude, pos.longitude)
    ]

    coords_array = np.array(all_coords)

    lats1, lons1 = coords_array[:-1].T
    lats2, lons2 = coords_array[1:].T

    distances = cs.GEODESIC.inv(lats1=lats1, lons1=lons1, lats2=lats2, lons2=lons2)[2]

    distances = [cs.meters_to_km(distance) for distance in distances]

    return distances


def path_to_dict(path: ci.Path, num_decimals: int = 4) -> dict[int, str]:
    """Converts a ci.Path msg to a dictionary suitable for printing.

    Args:
        path (ci.Path): The Path msg to be converted.
        num_decimals (int, optional): The number of decimal places to round to, default 4.

    Returns:
        dict[int, str]: Keys are the indices of the formatted latlon waypoints.
    """
    return {
        i: f"({waypoint.latitude:.{num_decimals}f}, {waypoint.longitude:.{num_decimals}f})"
        for i, waypoint in enumerate(path.waypoints)
    }


if __name__ == "__main__":
    main()
