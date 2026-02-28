"""Node to publish mock global path data.
The node is represented by the `MockGlobalPath` class."""

import rclpy
from rclpy.node import Node
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci
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
                ("interval_spacing", rclpy.Parameter.Type.DOUBLE),
                ("write", rclpy.Parameter.Type.BOOL),
                ("gps_threshold", rclpy.Parameter.Type.DOUBLE),
                ("test_plan", rclpy.Parameter.Type.STRING),
            ],
        )

        self.gps_sub = self.create_subscription(
            msg_type=ci.GPS, topic="gps", callback=self.global_path_callback, qos_profile=10
        )

        self.global_path_pub = self.create_publisher(
            msg_type=ci.Path, topic="global_path", qos_profile=10
        )

        test_plan = TestPlan(self.get_parameter("test_plan").get_parameter_value().string_value)
        self.global_path = test_plan.global_path
        self.pos = None

    # Timer callbacks
    def global_path_callback(self, gps: ci.GPS):
        """ """

        if self.should_update_gpath(gps):

            pos = self.pos

            # we need to obtain the actual distances between every waypoint in the global path
            path_spacing = gp.calculate_interval_spacing(pos, self.global_path.waypoints)
            interval_spacing = self.get_parameter("interval_spacing")._value

            # this checks if global path is just a destination point
            if len(self.global_path.waypoints) < 2:
                self.get_logger().debug(
                    f"Generating new path from {pos.latitude:.4f}, {pos.longitude:.4f} to "
                    f"{self.global_path.waypoints[0].latitude:.4f}, "
                    f"{self.global_path.waypoints[0].longitude:.4f}"
                )

                write = self.get_parameter("write")._value
                if write:
                    self.get_logger().debug("Writing generated path to new file")

                msg = gp.generate_path(
                    dest=self.global_path.waypoints[0],
                    interval_spacing=interval_spacing,
                    pos=pos,
                    write=write,
                )

            elif max(path_spacing) > interval_spacing:
                self.get_logger().debug(
                    f"Some waypoints in the global path exceed the maximum interval spacing of"
                    f" {interval_spacing} km. Interpolating between waypoints and generating path"
                )

                write = self.get_parameter("write")._value
                if write:
                    self.get_logger().debug("Writing generated path to new file")

                msg = gp._interpolate_path(
                    global_path=self.global_path,
                    interval_spacing=interval_spacing,
                    pos=pos,
                    path_spacing=path_spacing,
                    write=write,
                )

            else:
                msg = self.global_path

        else:
            # need to still publish the path even if its unchanged
            # to ensure that node_navigate will still receive the waypoints
            # in the event that this node launches first and publishes the
            # mock global path once into the void
            self.get_logger().debug("Publishing the global path unchanged")
            msg = self.global_path

        self.get_logger().debug(f"Publishing mock global path: {gp.path_to_dict(msg)}")
        self.global_path = msg
        self.global_path_pub.publish(msg)

    def should_update_gpath(self, gps: ci.GPS):
        """Determines if the gps location has changed enough since the last time this function
        was called to warrant updating the global path. Doing this ensures that polaris is never
        toofar away from the nearest global waypoint.

        Args:
            gps (ci.GPS): current GPS data from polaris

        Returns:
            bool: True if the gps location has changed enough to warrant an update of global path
        """
        update_gpath = False

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
                update_gpath = True
        else:
            # first time being called
            # should always force mock global path to update
            update_gpath = True

        self.pos = pos
        return update_gpath


if __name__ == "__main__":
    main()
