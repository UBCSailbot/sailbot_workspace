"""
This node observes the navigate node by subscribing to the local_path ROS
topic.
"""

from multiprocessing import Manager, Process

import custom_interfaces.msg as ci
import rclpy
from rclpy.node import Node

import local_pathfinding.visualizer as visualizer


def main():
    manager = Manager()
    queue = manager.Queue()

    ros_process = Process(target=ros_node, args=(queue,), daemon=True)
    dash_process = Process(target=visualizer.dash_app, args=(queue,), daemon=True)

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

        self.msgs = []
        self.prev_lpath_data = None
        self.queue = queue

    def local_path_callback(self, msg):
        if not self.is_msg_different(msg):
            self.get_logger().info("Message is the same as previous, skipping update.")
            return

        self.prev_lpath_data = msg
        self.msgs.append(msg)

        self.state = visualizer.VisualizerState(msgs=self.msgs)
        self.queue.put(self.state)

    def is_msg_different(self, msg):
        if self.prev_lpath_data is None:
            return True

        # Compare global_paths
        if self.prev_lpath_data.local_path.waypoints != msg.local_path.waypoints:
            return True
        if self.prev_lpath_data.global_path.waypoints != msg.global_path.waypoints:
            return True
        if self.prev_lpath_data.gps != msg.gps:
            return True

        return False


def ros_node(queue):
    rclpy.init(args=None)
    sailbot_observer = SailbotObserver(queue)

    try:
        rclpy.spin(node=sailbot_observer)
    except KeyboardInterrupt:
        sailbot_observer.get_logger().info("Keyboard interrupt [^C], shutting down.")
    finally:
        sailbot_observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
