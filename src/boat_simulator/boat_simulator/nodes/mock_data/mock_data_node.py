import random
import sys

import rclpy
from custom_interfaces.msg import DesiredHeading, HelperHeading, SailCmd
from rclpy.node import Node

import boat_simulator.common.constants as Constants



class MockDataNode(Node):
    """the purpose of this node is to publish to all topics that there is subscribers in physics engine node so send goal code can work.
    
    based on following:
    https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

     Publishers:
        desired_heading_pub (Publisher): Publishes GPS data in a `GPS` message.
        wind_sensors_pub (Publisher): Publishes mock desired heading data in a `DesiredHeading` message.
        sail_trim_tab_angle_pub (Publisher): Publishes mock sail trim tab angle data in a `SailCmd` message
    """
    def __init__(self):
        super().__init__("mock_data")
        self.__declare_ros_parameters()

        self.desired_heading_pub = self.create_publisher(
            msg_type=DesiredHeading,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.DESIRED_HEADING,
            qos_profile=self.qos_depth,
        )
        self.sail_trim_tab_angle_pub = self.create_publisher(
            msg_type=SailCmd,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.SAIL_TRIM_TAB_ANGLE,
            qos_profile=self.qos_depth,
        )

        timer_period = self.pub_period  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __declare_ros_parameters(self):
        """Declares ROS parameters from the global configuration file that will be used in this
        node. This node will monitor for any changes to these parameters during execution and will
        update itself accordingly.
        """
        self.get_logger().debug("Declaring ROS parameters...")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("mock_sail_trim_tab", rclpy.Parameter.Type.BOOL),
                ("mock_desired_heading", rclpy.Parameter.Type.BOOL),
                ("qos_depth", rclpy.Parameter.Type.INTEGER),
                ("mock_desired_heading_lower_bound", rclpy.Parameter.Type.DOUBLE),
                ("mock_desired_heading_upper_bound", rclpy.Parameter.Type.DOUBLE),
                ("mock_sail_trim_tab_lower_bound", rclpy.Parameter.Type.DOUBLE),
                ("mock_sail_trim_tab_upper_bound", rclpy.Parameter.Type.DOUBLE)
            ],
        )

        all_parameters = self._parameters
        for name, parameter in all_parameters.items():
            value_str = str(parameter.value)
            self.get_logger().debug(f"Got parameter {name} with value {value_str}")

    def timer_callback(self):
        if self.mock_desired_heading:
            self.publish_mock_desired_heading()
        if self.mock_sail_trim_tab:
            self.publish_mock_sail_trim_tab_angle()

    def publish_mock_desired_heading(self):
        """Publishes mock wind sensor data."""
        heading = random.uniform(self.mock_desired_heading_lower_bound, self.mock_desired_heading_upper_bound)

        helper_heading = HelperHeading()
        helper_heading.heading = heading

        msg = DesiredHeading()
        msg.heading = helper_heading

        self.desired_heading_pub.publish(msg)
        self.get_logger().info(
            f'Publishing to {self.desired_heading_pub.topic} '
            f'a mock desired heading of {heading} degrees'
        )

    def publish_mock_sail_trim_tab_angle(self):
        """Publishes mock wind sensor data."""
        trim_tab_angle_degrees = random.uniform(self.mock_sail_trim_tab_lower_bound, self.mock_sail_trim_tab_upper_bound)

        msg = SailCmd()
        msg.trim_tab_angle_degrees = trim_tab_angle_degrees

        self.sail_trim_tab_angle_pub.publish(msg)
        self.get_logger().info(
            f'Publishing to {self.sail_trim_tab_angle_pub.topic} '
            f'a mock trim tab angle of {trim_tab_angle_degrees} degrees'
        )

    @property
    def pub_period(self) -> float:
        return self.get_parameter("pub_period_sec").get_parameter_value().double_value

    @property
    def mock_desired_heading(self) -> bool:
        return self.get_parameter("mock_desired_heading").get_parameter_value().bool_value

    @property
    def mock_sail_trim_tab(self) -> bool:
        return self.get_parameter("mock_sail_trim_tab").get_parameter_value().bool_value

    @property
    def qos_depth(self) -> int:
        return self.get_parameter("qos_depth").get_parameter_value().integer_value


def main(args=None):
    rclpy.init(args=args)

    if is_mock_data_enabled():
        try:
            node = MockDataNode()
            rclpy.spin(node)
        finally:
            rclpy.shutdown()


def is_mock_data_enabled() -> bool:
    try:
        is_mock_data_enabled_index = sys.argv.index(Constants.MOCK_DATA_CLI_ARG_NAME) + 1
        is_mock_data_enabled = sys.argv[is_mock_data_enabled_index] == "true"
    except ValueError:
        is_mock_data_enabled = False
    return is_mock_data_enabled


if __name__ == "__main__":
    main()
