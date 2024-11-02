import random

import rclpy
from custom_interfaces.msg import DesiredHeading, HelperHeading, SailCmd
from rclpy.node import Node

import boat_simulator.common.constants as Constants

# based on following:
# https://docs.ros.org
# /en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
# the purpose of this node is to publish to all topics that there is
# subscribers in physics engine node so send goal code can work.


class MockDataNode(Node):

    def __init__(self):
        super().__init__("mock_data")

        self.desired_heading_pub = self.create_publisher(
            msg_type=DesiredHeading,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.DESIRED_HEADING,
            qos_profile=10,
        )
        self.sail_trim_tab_angle_pub = self.create_publisher(
            msg_type=SailCmd,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.SAIL_TRIM_TAB_ANGLE,
            qos_profile=10,
        )

        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().warn("MOCK DATA NODE ACTIVE")
        self.publish_mock_desired_heading()
        self.publish_mock_sail_trim_tab_angle()

    def publish_mock_desired_heading(self):
        """Publishes mock wind sensor data."""
        heading = random.uniform(-179.99, 180.0)

        helper_heading = HelperHeading()
        helper_heading.heading = heading

        msg = DesiredHeading()
        msg.heading = helper_heading

        self.desired_heading_pub.publish(msg)
        self.get_logger().info(
            f"Publishing to {self.desired_heading_pub.topic} "
            + f"a mock desired heading of {heading} degrees"
        )

    def publish_mock_sail_trim_tab_angle(self):
        """Publishes mock wind sensor data."""
        trim_tab_angle_degrees = random.uniform(-40, 40)

        msg = SailCmd()
        msg.trim_tab_angle_degrees = trim_tab_angle_degrees

        self.sail_trim_tab_angle_pub.publish(msg)
        self.get_logger().info(
            f"Publishing to {self.sail_trim_tab_angle_pub.topic} "
            + f"a mock trim tab angle of {trim_tab_angle_degrees} degrees"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockDataNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
