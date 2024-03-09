"""Node that publishes the mock global path, represented by the `MockGlobalPath` class."""

import csv
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from custom_interfaces.msg import GPS, HelperLatLon, Path
from rclpy.node import Node

from local_pathfinding.coord_systems import GEODESIC, meters_to_km

# Mock gps data to get things running until we have a running gps node
# TODO Remove when NET publishes GPS
MOCK_GPS = GPS(lat_lon=HelperLatLon(latitude=49.1154488073483, longitude=-125.95696431913618))


def main(args=None):
    rclpy.init(args=args)
    mock_global_path = MockGlobalPath()

    rclpy.spin(node=mock_global_path)

    mock_global_path.destroy_node()
    rclpy.shutdown()


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
        dest (Union[HelperLatLon, list[HelperLatLon]]): The destination point or partial path
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


def interpolate_path(
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


class MockGlobalPath(Node):
    """Stores and publishes the mock global path to the global_path topic.

    Subscribers:
        gps_sub (Subscription): Subscribe to a `GPS` msg which contains the current GPS location of
        sailbot.

    Publishers and their timers:
        global_path_pub (Publisher): Publishes a `Path` msg containing the global path
        global_path_timer (Timer): Periodically run the global path callback

    Attributes from subscribers:
        gps (GPS): Data from the GPS sensor

    Attributes:
        path_mod_tmstmp (Str): The modified timestamp of the global path csv file
        file_path (Str): The filepath of the global path csv file

    Parameters: see [Sailbot ROS Parameter Configuration](https://github.com/UBCSailbot/sailbot_workspace/blob/main/src/global_launch/config/README.md)  # noqa: E501
        for their documentation
    """

    def __init__(self):
        super().__init__(node_name="mock_global_path")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("global_path_filepath", rclpy.Parameter.Type.STRING),
                ("interval_spacing", rclpy.Parameter.Type.DOUBLE),
                ("write", rclpy.Parameter.Type.BOOL),
                ("gps_threshold", rclpy.Parameter.Type.DOUBLE),
                ("force", rclpy.Parameter.Type.BOOL),
            ],
        )

        # Subscribers
        self.gps_sub = self.create_subscription(
            msg_type=GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )

        # Publishers
        self.global_path_pub = self.create_publisher(
            msg_type=Path, topic="global_path", qos_profile=10
        )

        # Path callback timer
        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        self.get_logger().debug(f"Got parameter: {pub_period_sec=}")

        self.global_path_timer = self.create_timer(
            timer_period_sec=pub_period_sec,
            callback=self.global_path_callback,
        )

        # Attributes
        self.gps = MOCK_GPS  # TODO Remove when NET publishes GPS
        self.path_mod_tmstmp = None
        self.file_path = None

    # Subscriber callbacks
    def gps_callback(self, msg: GPS):
        """Store the gps data and check if the global path needs to be updated.

        If the position has changed by more than gps_threshold * interval_spacing since last step,
        the global_path_callback is run with the force parameter set to true, bypassing any checks.
        """
        self.get_logger().debug(f"Received data from {self.gps_sub.topic}: {msg}")

        position_delta = meters_to_km(
            GEODESIC.inv(
                lats1=self.gps.lat_lon.latitude,
                lons1=self.gps.lat_lon.longitude,
                lats2=msg.lat_lon.latitude,
                lons2=msg.lat_lon.longitude,
            )[2]
        )
        gps_threshold = self.get_parameter("gps_threshold")._value
        interval_spacing = self.get_parameter("interval_spacing")._value
        if position_delta > gps_threshold * interval_spacing:
            self.get_logger().info(
                f"GPS data changed by more than {gps_threshold*interval_spacing} km. Running ",
                "global path callback",
            )

            self.set_parameters([rclpy.Parameter("force", rclpy.Parameter.Type.BOOL, True)])
            self.global_path_callback()

        self.gps = msg

    # Timer callbacks
    def global_path_callback(self):
        """Check if the global path csv file has changed. If it has, the new path is published.

        This function is also called by the gps callback if the gps data has changed by more than
        gps_threshold.

        Depending on the boolean value of the write parameter, each generated path may be written
        to a new csv file in the same directory as the source csv file.

        Global path can be changed by modifying mock_global_path.csv or modifying the
        global_path_filepath parameter.

        """
        if not self._all_subs_active():
            self._log_inactive_subs_warning()

        file_path = self.get_parameter("global_path_filepath")._value

        # check when global path was changed last
        path_mod_tmstmp = time.ctime(os.path.getmtime(file_path))

        # check if the global path has been forced to update by a parameter change
        force = self.get_parameter("force")._value

        # Only publish path if the path has changed or gps has changed by more than gps_threshold
        if path_mod_tmstmp == self.path_mod_tmstmp and self.file_path == file_path and not force:
            return

        else:
            self.get_logger().info(
                f"Global path file is: {os.path.basename(file_path)}\n Reading path"
            )

            global_path = Path()

            with open(file_path, "r") as file:
                reader = csv.reader(file)
                # skip header
                reader.__next__()
                for row in reader:
                    global_path.waypoints.append(
                        HelperLatLon(latitude=float(row[0]), longitude=float(row[1]))
                    )

            pos = self.gps.lat_lon

            # obtain the actual distances between every waypoint in the global path
            path_spacing = calculate_interval_spacing(pos, global_path.waypoints)

            # obtain desired interval spacing
            interval_spacing = self.get_parameter("interval_spacing")._value

            # check if global path is just a destination point
            if len(global_path.waypoints) < 2:
                self.get_logger().info(
                    f"Generating new path from {pos.latitude:.4f}, {pos.longitude:.4f} to "
                    f"{global_path.waypoints[0].latitude:.4f}, "
                    f"{global_path.waypoints[0].longitude:.4f}"
                )

                write = self.get_parameter("write")._value
                if write:
                    self.get_logger().info("Writing generated path to new file")

                msg = generate_path(
                    dest=global_path.waypoints[0],
                    interval_spacing=interval_spacing,
                    pos=pos,
                    write=write,
                    file_path=file_path,
                )
            # Check if any waypoints are too far apart
            elif max(path_spacing) > interval_spacing:
                self.get_logger().info(
                    f"Some waypoints in the global path exceed the maximum interval spacing of"
                    f" {interval_spacing} km. Interpolating between waypoints and generating path"
                )

                write = self.get_parameter("write")._value
                if write:
                    self.get_logger().info("Writing generated path to new file")

                msg = interpolate_path(
                    global_path=global_path,
                    interval_spacing=interval_spacing,
                    pos=pos,
                    path_spacing=path_spacing,
                    write=write,
                    file_path=file_path,
                )

            else:
                msg = global_path

            # publish global path
            self.global_path_pub.publish(msg)
            self.get_logger().info(
                f"Publishing to {self.global_path_pub.topic}: {path_to_dict(msg)}"
            )

            self.set_parameters([rclpy.Parameter("force", rclpy.Parameter.Type.BOOL, False)])
            self.path_mod_tmstmp = path_mod_tmstmp
            self.file_path = file_path

    def _all_subs_active(self) -> bool:
        return self.gps is not None

    def _log_inactive_subs_warning(self):
        self.get_logger().warning("Waiting for GPS to be published")


if __name__ == "__main__":
    main()
