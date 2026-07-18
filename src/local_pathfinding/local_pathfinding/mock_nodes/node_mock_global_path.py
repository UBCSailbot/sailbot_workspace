"""Node to publish mock global path data.
The node is represented by the `MockGlobalPath` class."""

from pathlib import Path

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from test_plans.test_plan import TestPlan
from local_pathfinding.node_navigate import Sailbot

import custom_interfaces.msg as ci

DEFAULT_GLOBAL_PATH_CSV = (
    Path(__file__).resolve().parent / "mock_global_path.csv"
)


class ExceededTimeError(Exception):
    pass


def main(args=None):
    rclpy.init(args=args)
    mock_global_path = MockGlobalPath()
    mock_global_path.publish_global_path()

    rclpy.spin(node=mock_global_path)

    mock_global_path.destroy_node()
    rclpy.shutdown()


class MockGlobalPath(Node):
    """Stores and publishes the mock global path to the global_path topic.

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
                ("global_path_csv_path", str(DEFAULT_GLOBAL_PATH_CSV)),
            ],
        )

        self.global_path_pub = self.create_publisher(
            msg_type=ci.Path, topic="global_path", qos_profile=10
        )

        self._test_plan_name = self.get_parameter("test_plan").get_parameter_value().string_value
        self._global_path_csv_path = Path(
            self.get_parameter("global_path_csv_path").get_parameter_value().string_value
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.global_path = self._load_global_path()

    def _load_global_path(self, from_csv: bool = False) -> ci.Path:
        """Load the active mock global path"""
        if from_csv:
            global_path = Sailbot._read_global_path_from_file(self._global_path_csv_path)
            if global_path is None:
                self.get_logger().warning(
                    "Failed to load global path from CSV, keeping current path"
                )
                return self.global_path
            else:
                self.get_logger().debug(
                    f"Loaded mock global path from CSV: {self._global_path_csv_path}"
                )
                return global_path

        test_plan = TestPlan(self._test_plan_name)
        return test_plan.global_path

    def _on_set_parameters(self, params: list[Parameter]) -> SetParametersResult:
        """Reload the mock global path when the CSV path parameter is set."""
        should_reload = False
        for param in params:
            if param.name == "global_path_csv_path":
                self._global_path_csv_path = Path(param.value)
                should_reload = True

        if should_reload:
            self.global_path = self._load_global_path(from_csv=True)
            self.publish_global_path()

        return SetParametersResult(successful=True)

    # Publishing the global path
    def publish_global_path(self) -> None:
        # Publish the current global path exactly as defined in the CSV/test plan source. This node
        # does not interpolate between waypoints or generate a path from the boat's position.
        msg = self.global_path
        start_time_ns = self.get_clock().now().nanoseconds
        while (True):
            node_names = self.get_node_names()
            if 'navigate_main' in node_names:
                break
            if self.get_clock().now().nanoseconds - start_time_ns > 2e9:
                raise ExceededTimeError(
                    "Node mock global path waited for 2 seconds for navigate_main, but "
                    "navigate_main didn't become alive"
                )
        self.get_logger().info(f"Published mock global path: {Sailbot._path_to_dict(msg)}")
        self.global_path_pub.publish(msg)


if __name__ == "__main__":
    main()
