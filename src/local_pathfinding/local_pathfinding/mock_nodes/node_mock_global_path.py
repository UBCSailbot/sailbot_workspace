"""Node to publish mock global path data.
The node is represented by the `MockGlobalPath` class."""

import rclpy
from rclpy.node import Node
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci


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
                ("test_plan", rclpy.Parameter.Type.STRING),
            ],
        )

        self.gps_sub = self.create_subscription(
            msg_type=ci.GPS, topic="gps", callback=self.global_path_callback, qos_profile=10
        )

        self.global_path_pub = self.create_publisher(
            msg_type=ci.Path, topic="global_path", qos_profile=10
        )

        test_plan_name = self.get_parameter("test_plan").get_parameter_value().string_value
        self.get_logger().debug(f"Mock global path node test plan: {test_plan_name}")
        test_plan = TestPlan(test_plan_name)

        self.global_path = test_plan.global_path

    @staticmethod
    def _path_to_dict(path: ci.Path, num_decimals: int = 4) -> dict[int, str]:
        """Converts a ci.Path msg to a dictionary suitable for logging."""
        return {
            i: f"({waypoint.latitude:.{num_decimals}f}, {waypoint.longitude:.{num_decimals}f})"
            for i, waypoint in enumerate(path.waypoints)
        }

    # Timer callbacks
    def global_path_callback(self, gps: ci.GPS):
        # Publish the test plan's global path exactly as defined. This node does not interpolate
        # between waypoints or generate a path from the boat's position — it relies solely on the
        # waypoints provided in the test plan. The path is still published on every GPS message so
        # that node_navigate receives the waypoints even if this node launched first.
        msg = self.global_path
        self.get_logger().debug(f"Publishing mock global path: {self._path_to_dict(msg)}")
        self.global_path_pub.publish(msg)


if __name__ == "__main__":
    main()
