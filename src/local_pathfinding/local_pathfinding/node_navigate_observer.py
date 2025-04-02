"""
This node observes the navigate node by subscribing to the local_path ROS
topic.
"""

import custom_interfaces.msg as ci
import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    sailbot = SailbotObserver()

    rclpy.spin(node=sailbot)

    sailbot.destroy_node()
    rclpy.shutdown()


class SailbotObserver(Node):
    """Observes the Sailbot node, through the local_path topic, as it navigates.

    Subscribers:
        local_path_sub (Subscription): Subscribe to a `LPathData` msg.

    Attributes From Subscribers:
        local_path: (ci.LPathData
    """

    def __init__(self):
        super().__init__(node_name="navigate_observer")

        self.local_path_sub = self.create_subscription(
            msg_type=ci.LPathData,
            topic="local_path",
            callback=self.local_path_callback,
            qos_profile=10,
        )

    def local_path_callback(self, msg: ci.GPS):
        self.get_logger().debug(f"Received data from {self.local_path_sub.topic}: {msg}")
        self.local_path_data = msg


if __name__ == "__main__":
    main()
