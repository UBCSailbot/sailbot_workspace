"""
The main function of this file spawns two processes:
    1. The SailbotObserver node which observes the navigate node
    2. the Dash app to display the visualizer data

"""

from collections import deque
from multiprocessing import Manager, Process, Queue, Value
from typing import Deque

import custom_interfaces.msg as ci
import rclpy
from rclpy.node import Node

import local_pathfinding.visualizer as vz


def main():
    interprocess_manager = Manager()
    interprocess_queue = interprocess_manager.Queue()
    queue_size = interprocess_manager.Value("i", 0)

    ros_process = Process(target=ros_node, args=(interprocess_queue, queue_size), daemon=True)
    dash_process = Process(target=vz.dash_app, args=(interprocess_queue, queue_size), daemon=True)

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


def ros_node(queue: Queue, queue_size: Value):
    """Launches the observer node.

    Args:
        queue (Queue): Stores VisualizerState Objects for visualization in the dash app
        queue_size (Value): Contains the number of messages currently in the queue.
                            Has a lock by default.
    """
    rclpy.init(args=None)
    sailbot_observer = SailbotObserver(queue, queue_size)

    try:
        rclpy.spin(node=sailbot_observer)
    except KeyboardInterrupt:
        sailbot_observer.get_logger().info("Keyboard interrupt [^C], shutting down.")
    finally:
        sailbot_observer.destroy_node()
        rclpy.shutdown()


class SailbotObserver(Node):
    """Observes all the data that the navigate node is currently aware of and is using to produce
       a local path.

    Subscribers:
        local_path_sub (Subscription): Subscribes to the `LPathData` topic.

    Attributes From Subscribers:
        queue (Queue): An interprocess queue used to pipe local pathfinding data to the
                               dash app process
        msgs (deque): An ordinary queue for storing the last x messages received by the subscriber
    """

    def __init__(self, queue: Queue, queue_size: Value):
        super().__init__("navigate_observer")
        self.get_logger().info("SailbotObserver node initialized")

        self.local_path_sub = self.create_subscription(
            msg_type=ci.LPathData,
            topic="local_path",
            callback=self.local_path_callback,
            qos_profile=10,
        )
        self.msgs: Deque[ci.LPathData] = deque(maxlen=100)
        self.queue = queue
        self.queue_size = queue_size

    def local_path_callback(self, msg: ci.LPathData):
        self.get_logger().debug(f"Received new local path message: {msg}")
        self.msgs.append(msg)
        self.queue_size.value += 1
        self.queue.put(vz.VisualizerState(msgs=self.msgs))


if __name__ == "__main__":
    main()
