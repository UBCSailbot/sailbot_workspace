"""Node to publish mock global path data.
The node is represented by the `MockGlobalPath` class."""

import os
import time

import custom_interfaces.msg as ci
import rclpy
from rclpy.node import Node

import local_pathfinding.coord_systems as cs
import local_pathfinding.global_path as gp


def main(args=None):
    rclpy.init(args=args)
    mock_global_path = MockGlobalPath()

    rclpy.spin(node=mock_global_path)

    mock_global_path.destroy_node()
    rclpy.shutdown()


class MockGlobalPath(Node):
    """Stores and publishes the mock global path to the global_path topic.

    Subscriber:
        gps_sub (Subscription): Subscribe to a `GPS` msg which contains the current GPS location of
        sailbot.

    Publisher:
        global_path_pub (Publisher): Publishes a `Path` msg containing the global path

    Attributes:
        path_mod_tmstmp (Str): The modified timestamp of the global path csv file
        file_path (Str): The filepath of the global path csv file
        gps (GPS): Data from the GPS topic
        global_path (Path): The most recently published version of the global path.

    Parameters: see [Sailbot ROS Parameter Configuration](https://github.com/UBCSailbot/sailbot_workspace/blob/main/src/global_launch/config/README.md)  # noqa: E501
        for their documentation
    """

    def __init__(self):
        super().__init__(node_name="mock_global_path")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("mode", rclpy.Parameter.Type.STRING),
                ("interval_spacing", rclpy.Parameter.Type.DOUBLE),
                ("write", rclpy.Parameter.Type.BOOL),
                ("gps_threshold", rclpy.Parameter.Type.DOUBLE),
                ("force", rclpy.Parameter.Type.BOOL),
                ("test_plan", rclpy.Parameter.Type.STRING),
            ],
        )

        self.mode = self.get_parameter("mode").get_parameter_value().string_value

        # Subscriber
        self.gps_sub = self.create_subscription(
            msg_type=ci.GPS, topic="gps", callback=self.global_path_callback, qos_profile=10
        )

        # Publisher
        self.global_path_pub = self.create_publisher(
            msg_type=ci.Path, topic="global_path", qos_profile=10
        )

        # Attributes
        self.pos = None
        self.path_mod_tmstmp = None
        self.file_path = None
        self.global_path = ci.Path()

    # Timer callbacks
    def global_path_callback(self, gps: ci.GPS):
        """Publish the current or updated global path.

        Recomputes the path only when the source file changes, the GPS
        position moves enough to trigger a forced update, or the file path
        parameter changes.
        """
        if self.mode == "development":
            # Resolve test_plan which may be a bare name like "basic.yaml"
            test_plan = self.get_parameter("test_plan").get_parameter_value().string_value
            if os.path.isabs(test_plan):
                file_path = test_plan
            else:
                ros_workspace = os.getenv("ROS_WORKSPACE", "/workspaces/sailbot_workspace")
                file_path = os.path.join(
                    ros_workspace,
                    "src",
                    "local_pathfinding",
                    "test_plans",
                    test_plan,
                )
        else:
            file_path = self.get_parameter("deploy_filepath")._value

        path_mod_tmstmp = time.ctime(os.path.getmtime(file_path))

        # this function updates self.pos internally because it needs to compare
        # the new position to the current self.pos
        self.check_pos(gps)

        # parameters used when recomputing the path
        force = self.get_parameter("force")._value
        interval_spacing = self.get_parameter("interval_spacing")._value
        write = self.get_parameter("write")._value

        # Only calculate a new path if the path file or GPS has effectively changed
        if path_mod_tmstmp == self.path_mod_tmstmp and self.file_path == file_path and not force:
            # still publish the path even if it's unchanged to ensure
            # downstream nodes receive the waypoints
            self.get_logger().debug("Publishing the global path unchanged")
            msg = self.global_path
        else:
            msg = self._compute_new_global_path(
                file_path=file_path,
                interval_spacing=interval_spacing,
                write=write,
            )

        self.get_logger().debug(f"Publishing mock global path: {gp.path_to_dict(msg)}")
        self.global_path = msg
        self.global_path_pub.publish(msg)

        # reset all checks for next function call
        self.set_parameters([rclpy.Parameter("force", rclpy.Parameter.Type.BOOL, False)])
        self.path_mod_tmstmp = path_mod_tmstmp
        self.file_path = file_path

    def _compute_new_global_path(
        self,
        file_path: str,
        interval_spacing: float,
        write: bool,
    ) -> ci.Path:
        """Compute a new global path based on the current position and CSV file."""

        global_path = gp.get_path(file_path=file_path)
        pos = self.pos

        # obtain the actual distances between every waypoint in the global path
        path_spacing = gp.calculate_interval_spacing(pos, global_path.waypoints)

        # check if global path is just a destination point
        if len(global_path.waypoints) < 2:
            self.get_logger().debug(
                f"Generating new path from {pos.latitude:.4f}, {pos.longitude:.4f} to "
                f"{global_path.waypoints[0].latitude:.4f}, "
                f"{global_path.waypoints[0].longitude:.4f}"
            )

            if write:
                self.get_logger().debug("Writing generated path to new file")

            return gp.generate_path(
                dest=global_path.waypoints[0],
                interval_spacing=interval_spacing,
                pos=pos,
                write=write,
                file_path=file_path,
            )

        # Check if any waypoints are too far apart
        if max(path_spacing) > interval_spacing:
            self.get_logger().debug(
                f"Some waypoints in the global path exceed the maximum interval spacing of"
                f" {interval_spacing} km. Interpolating between waypoints and generating path"
            )

            if write:
                self.get_logger().debug("Writing generated path to new file")

            return gp._interpolate_path(
                global_path=global_path,
                interval_spacing=interval_spacing,
                pos=pos,
                path_spacing=path_spacing,
                write=write,
                file_path=file_path,
            )

        return global_path

    def check_pos(self, gps: ci.GPS):
        """Get the gps data and check if the global path needs to be updated.

        If the position has changed by more than gps_threshold * interval_spacing since last step,
        the force parameter set to true, bypassing any checks in the global_path_callback.
        """

        pos = gps.lat_lon
        if self.pos:
            position_delta = cs.meters_to_km(
                cs.GEODESIC.inv(
                    lats1=self.pos.latitude,
                    lons1=self.pos.longitude,
                    lats2=pos.latitude,
                    lons2=pos.longitude,
                )[2]
            )
            gps_threshold = self.get_parameter("gps_threshold")._value
            interval_spacing = self.get_parameter("interval_spacing")._value

            if position_delta > gps_threshold * interval_spacing:
                self.get_logger().debug(
                    f"GPS data changed by more than {gps_threshold*interval_spacing} km. Running "
                    "global path callback"
                )

                self.set_parameters([rclpy.Parameter("force", rclpy.Parameter.Type.BOOL, True)])
        else:
            # first time being called
            # should always force mock global path to update
            self.set_parameters([rclpy.Parameter("force", rclpy.Parameter.Type.BOOL, True)])

        self.pos = pos


if __name__ == "__main__":
    main()
