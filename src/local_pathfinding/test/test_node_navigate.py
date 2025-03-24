import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your actual GPS message type

from local_pathfinding.node_navigate import Sailbot


@pytest.fixture
def ros_test_node():
    """Fixture to initialize and tear down ROS."""
    rclpy.init()
    node = Node("test_node")
    yield node
    rclpy.shutdown()


def test_gps_subscription(ros_test_node):
    """Test that publishing a GPS message updates self.gps."""

    sailbot = Sailbot()  # Initialize your Sailbot node
    test_publisher = ros_test_node.create_publisher(String, "gps", 10)  # Use correct msg type
    rclpy.spin_once(sailbot, timeout_sec=0.1)  # Allow subscriber to initialize

    # Create a test GPS message
    test_msg = String()
    test_msg.data = "42.0, -71.0"  # Example data; replace with actual GPS format

    # Publish message and allow time for it to be processed
    test_publisher.publish(test_msg)

    # Wait and spin to process callbacks
    for _ in range(10):  # Wait up to 1 second in small steps
        rclpy.spin_once(sailbot, timeout_sec=0.1)
        if sailbot.gps == test_msg.data:
            break

    # Assert that the GPS attribute was updated correctly
    assert sailbot.gps == test_msg.data, f"Expected {test_msg.data}, got {sailbot.gps}"

    sailbot.destroy_node()
