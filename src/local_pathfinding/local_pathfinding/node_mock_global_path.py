"""Node loads in Sailbot's position via GET request, loads a global path from a csv file,
and posts the mock global path via a POST request.
The node is represented by the `MockGlobalPath` class."""

import os
import time

import rclpy
from custom_interfaces.msg import GPS, HelperLatLon
from rclpy.node import Node

from local_pathfinding.coord_systems import GEODESIC, meters_to_km
from local_pathfinding.global_path import (
    GPS_URL,
    PATH_URL,
    _interpolate_path,
    calculate_interval_spacing,
    generate_path,
    get_path,
    get_pos,
    path_to_dict,
    post_path,
)

# Mock gps data to get things running until we have a running gps node
# TODO Remove when NET publishes GPS
MOCK_GPS = GPS(lat_lon=HelperLatLon(latitude=49.1154488073483, longitude=-125.95696431913618))


def main(args=None):
    rclpy.init(args=args)
    mock_global_path = MockGlobalPath()

    rclpy.spin(node=mock_global_path)

    mock_global_path.destroy_node()
    rclpy.shutdown()


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
        # get the publishing period parameter to use for callbacks
        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        self.get_logger().debug(f"Got parameter: {pub_period_sec=}")

        # mock global path callback runs repeatedly on a timer
        self.global_path_timer = self.create_timer(
            timer_period_sec=pub_period_sec,
            callback=self.global_path_callback,
        )
        # Attributes
        self.pos = MOCK_GPS.lat_lon
        self.path_mod_tmstmp = None
        self.file_path = None
        self.period = pub_period_sec

    def check_pos(self):
        """Get the gps data and check if the global path needs to be updated.

        If the position has changed by more than gps_threshold * interval_spacing since last step,
        the force parameter set to true, bypassing any checks in the global_path_callback.
        """
        self.get_logger().info(
            f"Retreiving current position from {GPS_URL}", throttle_duration_sec=1
        )

        pos = get_pos()
        if pos is None:
            return  # error is logged in calling function

        position_delta = meters_to_km(
            GEODESIC.inv(
                lats1=self.pos.latitude,
                lons1=self.pos.longitude,
                lats2=pos.latitude,
                lons2=pos.longitude,
            )[2]
        )
        gps_threshold = self.get_parameter("gps_threshold")._value
        interval_spacing = self.get_parameter("interval_spacing")._value

        if position_delta > gps_threshold * interval_spacing:
            self.get_logger().info(
                f"GPS data changed by more than {gps_threshold*interval_spacing} km. Running "
                "global path callback"
            )

            self.set_parameters([rclpy.Parameter("force", rclpy.Parameter.Type.BOOL, True)])

        self.pos = pos

    # Timer callbacks
    def global_path_callback(self):
        """Check if the global path csv file has changed. If it has, the new path is published.

        This function also checks if the gps data has changed by more than
        gps_threshold. If it has, the force parameter is set to true, bypassing any checks and
        updating the path.

        Depending on the boolean value of the write parameter, each generated path may be written
        to a new csv file in the same directory as the source csv file.

        Global path can be changed by modifying mock_global_path.csv or modifying the
        global_path_filepath parameter.

        """

        file_path = self.get_parameter("global_path_filepath")._value

        # check when global path was changed last
        path_mod_tmstmp = time.ctime(os.path.getmtime(file_path))

        self.check_pos()

        if self.pos is None:
            self.log_no_pos()
            return

        # check if the global path has been forced to update by a parameter change
        force = self.get_parameter("force")._value

        # Only publish path if the path has changed or gps has changed by more than gps_threshold
        if path_mod_tmstmp == self.path_mod_tmstmp and self.file_path == file_path and not force:
            return

        self.get_logger().info(
            f"Global path file is: {os.path.basename(file_path)}\n Reading path"
        )
        global_path = get_path(file_path=file_path)

        pos = self.pos

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

            msg = _interpolate_path(
                global_path=global_path,
                interval_spacing=interval_spacing,
                pos=pos,
                path_spacing=path_spacing,
                write=write,
                file_path=file_path,
            )

        else:
            msg = global_path

        # post global path
        if post_path(msg):
            self.get_logger().info(f"Posting path to {PATH_URL}: {path_to_dict(msg)}")
            self.set_parameters([rclpy.Parameter("force", rclpy.Parameter.Type.BOOL, False)])
            self.path_mod_tmstmp = path_mod_tmstmp
            self.file_path = file_path
        else:
            self.log_failed_post()

    def log_no_pos(self):
        self.get_logger().warn(
            f"Failed to get position from {GPS_URL} will retry in {self.period} seconds."
        )

    def log_failed_post(self):
        self.get_logger().warn(f"Failed to post path to {PATH_URL}")


if __name__ == "__main__":
    main()
