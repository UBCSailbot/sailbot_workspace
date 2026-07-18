"""
The main function of this file spawns two processes:
    1. The SailbotObserver node which observes the navigate node
    2. the Dash app to display the visualizer data

"""

import math
from collections import deque
from multiprocessing import Manager, Process, Queue
from typing import Deque, Union

import rclpy
from rclpy.node import Node

import custom_interfaces.msg as ci
import local_pathfinding.visualizer as vz


def main():
    interprocess_manager = Manager()
    interprocess_queue = interprocess_manager.Queue()

    ros_process = Process(target=ros_node, args=(interprocess_queue,), daemon=True)
    dash_process = Process(target=vz.dash_app, args=(interprocess_queue,), daemon=True)

    ros_process.start()
    dash_process.start()

    try:
        while ros_process.is_alive():
            dash_process.join(timeout=1)
            if dash_process.is_alive():
                continue
            exitcode = dash_process.exitcode
            ros_process.join(timeout=1)
            if not ros_process.is_alive():
                break
            print(f"Visualizer exited with code {exitcode}. Restarting...")
            dash_process = Process(target=vz.dash_app, args=(interprocess_queue,), daemon=True)
            dash_process.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt [^C], shutting down.")
    finally:
        ros_process.terminate()
        dash_process.terminate()
        ros_process.join()
        dash_process.join()


def ros_node(queue: Queue):
    """Launches the observer node.

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
    """Observes all the data that the navigate node is currently aware of and is using to produce
       a local path.

    Subscribers:
        local_path_sub (Subscription): Subscribes to the `LPathData` topic.
        heading_sub (Subscription): Subscribes to e-compass boat heading on `rudder`.

    Attributes From Subscribers:
        queue (Queue): An interprocess queue used to pipe local pathfinding data to the
                               dash app process
        msgs (deque): An ordinary queue for storing the last x messages received by the subscriber
    """

    def __init__(self, queue: Queue):
        super().__init__("navigate_observer")
        self.get_logger().info("SailbotObserver node initialized")

        self.local_path_sub = self.create_subscription(
            msg_type=ci.LPathData,
            topic="local_path",
            callback=self.local_path_callback,
            qos_profile=10,
        )
        self.heading_sub = self.create_subscription(
            msg_type=ci.HelperHeading,
            topic="rudder",
            callback=self.heading_callback,
            qos_profile=10,
        )
        self.msgs: Deque[ci.LPathData] = deque(maxlen=100)
        self.queue = queue
        self.msg: Union[ci.LPathData, None] = None
        self.heading: Union[ci.HelperHeading, None] = None
        self.last_replan_reason = ""

        self.create_timer(3.0, self.update_queue)

    def local_path_callback(self, msg: ci.LPathData):
        """Callback which stores the latest message

        Args:
            msg (ci.LPathData): Contains all the Local Path Data for Visualization
        """

        # self.get_logger().debug(f"Received new local path message: {msg}")
        self.msg = msg
        if msg.replan_reason:
            self.last_replan_reason = msg.replan_reason

        self.msgs.append(self.msg)

        if self.queue.qsize() < 1:
            self.update_queue()

    def heading_callback(self, msg: ci.HelperHeading) -> None:
        """Store a valid e-compass boat heading for visualization."""

        if not math.isfinite(msg.heading) or not (-180.0 < msg.heading <= 180.0):
            self.get_logger().warning(
                f"Ignoring invalid heading from {self.heading_sub.topic}: {msg.heading}. "
                "Expected a finite value in (-180, 180]."
            )
            return
        self.heading = msg

    def update_queue(self):
        """Send the latest state through the pipe to the dash app"""

        if self.msg is None:
            self.get_logger().warn("No message received by /local_path data has been received")
            return
        if self.heading is None:
            self.get_logger().warn("No e-compass boat heading has been received from /rudder")
            return

        if self.queue.qsize() >= 1:
            self.get_logger().debug(
                f"queue size is already {self.queue.qsize()}, "
                f"not sending another new visualizer state until this is consumed."
            )
            return

        self.queue.put(
            vz.VisualizerState(
                msgs=self.msgs,
                heading=self.heading,
                last_replan_reason=self.last_replan_reason,
            )
        )
        self.get_logger().debug(f"sent new visualizer state with {len(self.msgs)} messages.")
        self.get_logger().debug(f"queue: {self.queue.qsize()}")


if __name__ == "__main__":
    main()
