"""
This node observes the navigate node by subscribing to the local_path ROS
topic.
"""

from typing import List

import custom_interfaces.msg as ci
import plotly.graph_objects as go
import rclpy
from rclpy.node import Node

from local_pathfinding.coord_systems import latlon_to_xy


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
        local_path: (ci.LPathData)
    """

    def __init__(self):
        super().__init__(node_name="navigate_observer")

        self.local_path_sub = self.create_subscription(
            msg_type=ci.LPathData,
            topic="local_path",
            callback=self.local_path_callback,
            qos_profile=10,
        )

        self.msgs = list()  # TODO: Use Deque to quickly append new msgs
        self.prev_lpath_data = None

        self.get_logger().info("Plotted")

    def local_path_callback(self, msg: ci.GPS):

        if not self.is_msg_different(msg):
            self.get_logger().debug("Message is the same as the previous one. Skipping... ")

        self.get_logger().debug(f"Received data from {self.local_path_sub.topic}: {msg}")

        self.msgs.append(msg)
        self.prev_lpath_data = msg

        # TODO: Check if we need to update visualizer plot
        visualizer = Visualizer(self.msgs)
        visualizer.plot()

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


class Visualizer:
    def __init__(self, msgs: List[ci.LPathData]):
        self.local_path_data = msgs[-1]

        if not self.local_path_data:
            raise ValueError("local_path must not be None")

        if not self.local_path_data.gps:
            raise ValueError("gps must not be None")
        self.sailbot_pos_lat_lon = [msg.gps.lat_lon for msg in msgs]

        if not self.local_path_data.local_path:
            raise ValueError("local path must not be None")
        self.all_local_waypoints = [msg.local_path.waypoints for msg in msgs]

        if not self.local_path_data.global_path:
            raise ValueError("global path must not be None")

        self.all_global_waypoints = [msg.global_path.waypoints for msg in msgs]

        self.global_path = self.local_path_data.global_path
        self.reference_latlon = self.global_path.waypoints[-1]

        self.sailbot_pos_xy = [
            latlon_to_xy(reference=self.reference_latlon, latlon=sailbot_pos)
            for sailbot_pos in self.sailbot_pos_lat_lon
        ]  # TODO: Get a list of sailbot gps positions from the start of the simulation

        self.waypoints_pos_xy = [
            latlon_to_xy(reference=self.reference_latlon, latlon=waypoint)
            for waypoints in self.all_local_waypoints
            for waypoint in waypoints
        ]

        self.sailbot_pos_x = [pos.x for pos in self.sailbot_pos_xy]
        self.sailbot_pos_y = [pos.y for pos in self.sailbot_pos_xy]

        self.local_waypoints_x = [point.x for point in self.waypoints_pos_xy]
        self.local_waypoints_y = [point.y for point in self.waypoints_pos_xy]

    def plot(self):
        # Initial plot
        center_index = len(self.waypoints_pos_xy[-1]) // 2
        num_current_waypoints = len(self.local_waypoints_x)
        num_sailbot_positions = len(self.sailbot_pos_x)
        initial_boat_state = go.Scatter(
            x=[self.sailbot_pos_x[0]],
            y=[self.sailbot_pos_y[0]],
            mode="markers",
            marker=go.scatter.Marker(size=14, color="red"),
            text=["Boat"],
            name="Boat",
        )
        initial_state = [
            go.Scatter(
                x=[self.local_waypoints_x[0]],
                y=[self.local_waypoints_y[0]],
                mode="markers",
                marker=go.scatter.Marker(size=14),
                text=["Start"],
                name="Start",
            ),
            go.Scatter(
                x=self.local_waypoints_x[1 : num_current_waypoints - 1],
                y=self.local_waypoints_y[1 : num_current_waypoints - 1],
                mode="markers",
                marker=go.scatter.Marker(size=14),
                text=["Intermediate"] * (num_current_waypoints - 2),
                name="Intermediate",
            ),
            go.Scatter(
                x=[self.local_waypoints_x[-1]],
                y=[self.local_waypoints_y[-1]],
                mode="markers",
                marker=go.scatter.Marker(size=14),
                text=["Goal"] * (num_current_waypoints - 2),
                name="Goal",
            ),
        ]
        new_frames = [
            go.Frame(
                data=initial_state
                + [
                    go.Scatter(
                        x=[self.sailbot_pos_x[0]],
                        y=[self.sailbot_pos_y[0]],
                        mode="markers",
                        marker=go.scatter.Marker(size=14, color="red"),
                        text=["Boat"],
                        name="Boat",
                    )
                ],
                name=f"Boat {i}",
            )
            for i in range(0, num_sailbot_positions)
        ]
        fig = go.Figure(
            data=initial_state + [initial_boat_state],
            layout=go.Layout(
                mapbox_style="open-street-map",
                mapbox_zoom=7,
                mapbox_center={
                    "lat": self.local_waypoints_x[center_index],
                    "lon": self.local_waypoints_y[center_index],
                },
                margin={"r": 0, "t": 0, "l": 0, "b": 0},
                xaxis=dict(title="X"),
                yaxis=dict(title="Y"),
                updatemenus=[
                    dict(
                        type="buttons",
                        showactive=False,
                        buttons=[
                            dict(
                                label="Play",
                                method="animate",
                                args=[
                                    None,
                                    {
                                        "frame": {"duration": 1000, "redraw": True},
                                        "mode": "immediate",
                                    },
                                ],
                            ),
                            dict(
                                label="Pause",
                                method="animate",
                                args=[
                                    [None],
                                    {
                                        "frame": {"duration": 0, "redraw": True},
                                        "mode": "immediate",
                                    },
                                ],
                            ),
                        ],
                    )
                ],
            ),
            frames=new_frames,
        )

        fig.show()


if __name__ == "__main__":
    main()
