"""The Global Path Module, which retrieves the global path from a specified http source and
sends it to NET via POST request.

The main function accepts two CLI arguments:
    file_path (str): The path to the global path csv file.
    --interval (float, Optional): The desired path interval length in km.
"""

import argparse
import csv
import json
import os
import time
from datetime import datetime
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

import numpy as np
from custom_interfaces.msg import HelperLatLon, Path

from local_pathfinding.coord_systems import GEODESIC, meters_to_km

GPS_URL = "http://localhost:3005/api/gps"
PATH_URL = "http://localhost:8081/global-path"
GLOBAL_PATHS_FILE_PATH = "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths"
PERIOD = 5  # seconds


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path", help="The path to the global path csv file.")
    parser.add_argument("--interval", help="Desired path interval length.", type=float)
    args = parser.parse_args()

    file_path = args.file_path
    path_mod_tmstmp = None
    pos = None

    try:
        path = get_path(file_path)
        print(f"retrieved path from {file_path}", path_to_dict(path))
    except FileNotFoundError:
        print(f"{file_path} not found. Please enter a valid file path.")
        exit(1)

    # Main service loop
    while True:
        time.sleep(PERIOD)
        timestamp = time.ctime(os.path.getmtime(file_path))

        # We should try to retrieve the position on every loop
        pos = get_pos()

        if pos is None:
            print(f"Failed to retrieve position from {GPS_URL}")
            continue

        position_delta = meters_to_km(
            GEODESIC.inv(
                lats1=pos.latitude,
                lons1=pos.longitude,
                lats2=path.waypoints[0].latitude,
                lons2=path.waypoints[0].longitude,
            )[2]
        )

        # exit loop if the path has not been modified or interval lengths are fine
        if (timestamp == path_mod_tmstmp) and (
            (args.interval is None) or position_delta <= args.interval
        ):
            continue

        if args.interval is not None:
            # interpolate path will interpolate new path and save it to a new csv file
            path = interpolate_path(
                path=path,
                pos=pos,
                interval_spacing=args.interval,
                file_path=file_path,
            )

        if post_path(path):
            print("Global path successfully updated.")
            print(f"position was {pos}")
            file_path = get_most_recent_file(GLOBAL_PATHS_FILE_PATH)
            timestamp = time.ctime(os.path.getmtime(file_path))
        else:
            # if the post was unsuccessful, we should try again
            # so don't update the timestamp
            continue

        path_mod_tmstmp = timestamp


def get_most_recent_file(directory_path: str) -> str:
    """
    Returns the most recently modified file in the specified directory.

    Args:
        directory_path (str): The path to the directory containing the files.

    Returns:
        str: The path to the most recently modified file.
    """
    all_files = os.listdir(directory_path)

    # Filter out directories and get the full file paths
    files = [
        os.path.join(directory_path, file)
        for file in all_files
        if os.path.isfile(os.path.join(directory_path, file))
    ]

    # Sort the files based on their last modification time
    files.sort(key=lambda x: os.path.getmtime(x), reverse=True)

    if files:
        return files[0]
    else:
        return ""


def get_path(file_path: str) -> Path:
    """Returns the global path from the specified file path.

    Args:
        file_path (str): The path to the global path csv file.

    Returns:
        (Path): The global path retrieved from the csv file.
    """
    path = Path()

    with open(file_path, "r") as file:
        reader = csv.reader(file)
        # skip header
        reader.__next__()
        for row in reader:
            path.waypoints.append(HelperLatLon(latitude=float(row[0]), longitude=float(row[1])))
    return path


def post_path(path: Path) -> bool:
    """Sends the global path to NET via POST request.

    Args:
        path (Path): The global path.

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
        urlopen(PATH_URL, json_data)
        return True
    except HTTPError as http_error:
        print(f"HTTP Error: {http_error.code}")
    except URLError as url_error:
        print(f"URL Error: {url_error.reason}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    return False


def get_pos() -> HelperLatLon:
    """Returns the current position of sailbot, retrieved from the an http GET request.

    Returns:
        HelperLatLon: The current position of sailbot
            OR
        None: If the position could not be retrieved.
    """
    try:
        position = json.loads(urlopen(GPS_URL).read())
    except HTTPError as http_error:
        print(f"HTTP Error: {http_error.code}")
        return None
    except URLError as url_error:
        print(f"URL Error: {url_error.reason}")
        return None
    except ConnectionResetError as connect_error:
        print(f"Connection Reset Error: {connect_error}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

    if len(position["data"]) == 0:
        print(f"Connection to {GPS_URL} successful. No position data available.")
        return None

    latitude = position["data"][-1]["latitude"]
    longitude = position["data"][-1]["longitude"]
    pos = HelperLatLon(latitude=latitude, longitude=longitude)

    return pos


def generate_path(
    dest: HelperLatLon,
    interval_spacing: float,
    pos: HelperLatLon,
    write: bool = False,
    file_path: str = "",
) -> Path:
    """Returns a path from the current GPS location to the destination point.
    Waypoints are evenly spaced along the path according to the interval_spacing parameter.
    Path does not include pos, but does include dest as the final element.

    If write is True, the path is written to a new csv file in the same directory as file_path,
    with the name of the original file, appended with a timestamp.

    Args:
        dest (HelperLatLon): The destination point
        interval_spacing (float): The desired distance between waypoints on the path
        pos (HelperLatLon): The current GPS location
        write (bool, optional): Whether to write the path to a new csv file, default False
        file_path (str, optional): The filepath to the global path csv file, default empty

    Returns:
        Path: The generated path
    """
    global_path = Path()

    lat1 = pos.latitude
    lon1 = pos.longitude

    lat2 = dest.latitude
    lon2 = dest.longitude

    distance = meters_to_km(GEODESIC.inv(lats1=lat1, lons1=lon1, lats2=lat2, lons2=lon2)[2])

    # minimum number of waypoints to not exceed interval_spacing
    n = np.floor(distance / interval_spacing)
    n = max(1, n)

    # npts returns a path with neither pos nor dest included
    global_path_tuples = GEODESIC.npts(lon1=lon1, lat1=lat1, lon2=lon2, lat2=lat2, npts=n)

    # npts returns (lon,lat) tuples, its backwards for some reason
    for lon, lat in global_path_tuples:
        global_path.waypoints.append(HelperLatLon(latitude=lat, longitude=lon))

    # append the destination point
    global_path.waypoints.append(HelperLatLon(latitude=lat2, longitude=lon2))

    if write:
        write_to_file(file_path=file_path, global_path=global_path)

    return global_path


def _interpolate_path(
    global_path: Path,
    interval_spacing: float,
    pos: HelperLatLon,
    path_spacing: list[float],
    write: bool = False,
    file_path: str = "",
) -> Path:
    """Interpolates and inserts subpaths between any waypoints which are spaced too far apart.

    Args:
        global_path (Path): The path to interpolate between
        interval_spacing (float): The desired spacing between waypoints
        pos (HelperLatLon): The current GPS location
        path_spacing (list[float]): The distances between pairs of points in global_path
        write (bool, optional): Whether to write the path to a new csv file, default False
        file_path (str, optional): The filepath to the global path csv file, default empty

    Returns:
        Path: The interpolated path
    """

    waypoints = [pos] + global_path.waypoints

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
    waypoints.pop(0)

    global_path.waypoints = waypoints

    if write:
        write_to_file(file_path=file_path, global_path=global_path)

    return global_path


def interpolate_path(
    path: Path,
    pos: HelperLatLon,
    interval_spacing: float,
    file_path: str,
    write=True,
) -> Path:
    """Interpolates path to ensure the interval lengths are less than or equal to the specified
    interval spacing.

    Args:
        path (Path): The global path.
        pos (HelperLatLon): The current position of the vehicle.
        interval_spacing (float): The desired interval spacing.
        file_path (str): The path to the global path csv file.
        write (bool, optional): Whether or not to write the new path to a csv file. Default True.

    Returns:
        Path: The interpolated path.
    """

    # obtain the actual distances between every waypoint in the path
    path_spacing = calculate_interval_spacing(pos, path.waypoints)

    # check if global path is just a destination point
    if len(path.waypoints) < 2:
        path = generate_path(
            dest=path.waypoints[0],
            interval_spacing=interval_spacing,
            pos=pos,
            write=write,
            file_path=file_path,
        )
    # Check if any waypoints are too far apart
    elif max(path_spacing) > interval_spacing:
        path = _interpolate_path(
            global_path=path,
            interval_spacing=interval_spacing,
            pos=pos,
            path_spacing=path_spacing,
            write=write,
            file_path=file_path,
        )

    return path


def calculate_interval_spacing(pos: HelperLatLon, waypoints: list[HelperLatLon]) -> list[float]:
    """Returns the distances between pairs of points in a list of latitudes and longitudes,
    including pos as the first point.

    Args:
        pos (HelperLatLon): The gps position of the boat
        waypoints (list[HelperLatLon]): The list of waypoints

    Returns:
        list[float]: The distances between pairs of points in waypoints [km]
    """
    all_coords = [(pos.latitude, pos.longitude)] + [
        (waypoint.latitude, waypoint.longitude) for waypoint in waypoints
    ]

    coords_array = np.array(all_coords)

    lats1, lons1 = coords_array[:-1].T
    lats2, lons2 = coords_array[1:].T

    distances = GEODESIC.inv(lats1=lats1, lons1=lons1, lats2=lats2, lons2=lons2)[2]

    distances = [meters_to_km(distance) for distance in distances]

    return distances


def write_to_file(file_path: str, global_path: Path, tmstmp: bool = True) -> Path:
    """Writes the global path to a new, timestamped csv file.

    Args
        file_path (str): The filepath to the global path csv file
        global_path (Path): The global path to write to file
        tmstmp (bool, optional): Whether to append a timestamp to the file name, default True

    Raises:
        ValueError: If file_path is not to an existing `global_paths` directory
    """

    # check if file_path is a valid file path
    if not os.path.isdir(os.path.dirname(file_path)) or not str(
        os.path.dirname(file_path)
    ).endswith("global_paths"):
        raise ValueError(f"Invalid file path: {file_path}")

    if tmstmp:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        dst_file_path = file_path.removesuffix(".csv") + f"_{timestamp}.csv"
    else:
        dst_file_path = file_path

    with open(dst_file_path, "w") as file:
        writer = csv.writer(file)
        writer.writerow(["latitude", "longitude"])
        for waypoint in global_path.waypoints:
            writer.writerow([waypoint.latitude, waypoint.longitude])


def path_to_dict(path: Path, num_decimals: int = 4) -> dict[int, str]:
    """Converts a Path msg to a dictionary suitable for printing.

    Args:
        path (Path): The Path msg to be converted.
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
