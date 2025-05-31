"""
Creates two processes, one for the ROS node and one for the Dash app.
The ROS node subscribes to the local_path topic and the Dash app visualizes the data.
"""

from collections import deque
from multiprocessing import Manager, Process

import custom_interfaces.msg as ci
import rclpy
from rclpy.node import Node

import local_pathfinding.visualizer as vz


def ros_node(queue):
    """Spins up the ROS node.

    Args:
        queue (Manager.Queue): Stores VisualizerState Objects for visualization in the dash app
    """
    rclpy.init(args=None)
    sailbot_observer = SailbotObserver(queue)

    try:
        rclpy.spin(node=sailbot_observer)
    except KeyboardInterrupt:
        sailbot_observer.get_logger().info("Keyboard interrupt [^C], shutting down.")
    finally:
        sailbot_observer.destroy_node()
        rclpy.shutdown()


class SailbotObserver(Node):
    """Observes the Sailbot node, through the local_path topic, as it navigates.

    Subscribers:
        local_path_sub (Subscription): Subscribe to a `LPathData` msg.

    Attributes From Subscribers:
        local_path: (ci.LPathData)
    """

    def __init__(self, queue):
        super().__init__("navigate_observer")
        self.get_logger().info("SailbotObserver node initialized")

        # Simple subscription to verify callback
        self.local_path_sub = self.create_subscription(
            ci.LPathData,  # Ensure this matches the actual message type
            "local_path",  # Ensure the topic name is correct
            self.local_path_callback,
            10,  # QoS profile
        )

        self.msgs = deque(maxlen=100)  # Store the last 100 messages
        self.prev_msg = None
        self.queue = queue

    def local_path_callback(self, msg):
        if not self.is_msg_different(msg):
            self.get_logger().debug("Message is the same as previous, skipping update.")
            return

        self.prev_msg = msg
        self.msgs.append(msg)

        self.get_logger().debug(f"Received new local path message: {msg}")

        self.state = vz.VisualizerState(msgs=self.msgs)
        self.queue.put(self.state)

    # helper functions
    def is_msg_different(self, msg) -> bool:
        """Check if the current message is different from the previous one.
        This is used to avoid sending the same message multiple times.

        Args:
            msg (ci.LPathData): The current message to compare.

        Returns:
            bool: True if the message is different, False otherwise.
        """
        if self.prev_msg is None:
            return True

        # Compare previous messages to the current messages
        if self.prev_msg.local_path.waypoints != msg.local_path.waypoints:
            return True
        if self.prev_msg.global_path.waypoints != msg.global_path.waypoints:
            return True
        if self.prev_msg.gps != msg.gps:
            return True
        # TODO: Add more comparisons when checking other ROS message attributes

        return False


def main():
    manager = Manager()  # Create a manager object to share data between processes
    queue = manager.Queue()

    ros_process = Process(target=ros_node, args=(queue,), daemon=True)
    dash_process = Process(target=vz.dash_app, args=(queue,), daemon=True)

    ros_process.start()
    dash_process.start()

    try:
        ros_process.join()
        dash_process.join()
    except KeyboardInterrupt:
        print("Keyboard interrupt [^C], shutting down.")
        ros_process.terminate()
        dash_process.terminate()
        ros_process.join()
        dash_process.join()


if __name__ == "__main__":
    main()
