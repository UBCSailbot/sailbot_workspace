"""
This node observes the navigate node by subscribing to the local_path ROS
topic.
"""

from multiprocessing import Process, Queue
from typing import List

import custom_interfaces.msg as ci
import dash
import plotly
import plotly.graph_objects as go
import rclpy
from dash import dcc, html
from dash.dependencies import Input, Output
from plotly.io._base_renderers import BaseHTTPRequestHandler, HTTPServer
from plotly.validators.scatter.marker import SymbolValidator
from rclpy.node import Node

from local_pathfinding.coord_systems import latlon_to_xy

queue = Queue()
# Create the Dash app
app = dash.Dash(__name__)


def main():
    ros_process = Process(target=ros_node, daemon=True)
    dash_process = Process(target=dash_app, daemon=True)

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


class VisualizerState:
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

        self.all_waypoints_pos_xy = [
            [
                latlon_to_xy(reference=self.reference_latlon, latlon=waypoint)
                for waypoint in waypoints
            ]
            for waypoints in self.all_local_waypoints
        ]

        self.last_waypoints_pos_xy = [
            latlon_to_xy(reference=self.reference_latlon, latlon=waypoint)
            for waypoint in self.all_local_waypoints[-1]
        ]

        self.sailbot_pos_x = [pos.x for pos in self.sailbot_pos_xy]
        self.sailbot_pos_y = [pos.y for pos in self.sailbot_pos_xy]

        self.last_local_waypoints_x = [point.x for point in self.last_waypoints_pos_xy]
        self.last_local_waypoints_y = [point.y for point in self.last_waypoints_pos_xy]

        self.all_local_waypoints_x = [
            point.x for waypoints in self.all_waypoints_pos_xy for point in waypoints
        ]

        self.all_local_waypoints_y = [
            point.y for waypoints in self.all_waypoints_pos_xy for point in waypoints
        ]


class SailbotObserver(Node):
    """Observes the Sailbot node, through the local_path topic, as it navigates.

    Subscribers:
        local_path_sub (Subscription): Subscribe to a `LPathData` msg.

    Attributes From Subscribers:
        local_path: (ci.LPathData)
    """

    def __init__(self):
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

    def local_path_callback(self, msg):
        if not self.is_msg_different(msg):
            self.get_logger().info("Message is the same as previous, skipping update.")
            return


        self.prev_lpath_data = msg
        self.msgs.append(msg)

        self.state = VisualizerState(msgs=self.msgs)
        queue.put(self.state)

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

    # def final_plot(self, state: VisualizerState):
    #     if state is None:
    #         self.get_logger().info("No state to plot.")

    #     # Initial plot
    #     center_index = len(state.all_waypoints_pos_xy[-1]) // 2
    #     num_waypoints = len(state.all_waypoints_pos_xy[-1])
    #     initial_boat_state = go.Scatter(
    #         x=[state.sailbot_pos_x[0]],
    #         y=[state.sailbot_pos_y[0]],
    #         mode="markers",
    #         marker_symbol=self.raw_symbols,
    #         marker_line_color="darkseagreen",
    #         marker_color="lightgreen",
    #         marker_line_width=2,
    #         marker_size=15,
    #         text=["Boat"],
    #         name="Boat",
    #     )
    #     initial_state = [
    #         go.Scatter(
    #             x=[state.all_local_waypoints_x[0]],
    #             y=[state.all_local_waypoints_y[0]],
    #             mode="markers",
    #             marker=go.scatter.Marker(size=14),
    #             text=["Start"],
    #             name="Start",
    #         ),
    #         go.Scatter(
    #             x=state.all_local_waypoints_x[1 : num_waypoints - 1],
    #             y=state.all_local_waypoints_y[1 : num_waypoints - 1],
    #             mode="markers",
    #             marker=go.scatter.Marker(size=14),
    #             text=["Intermediate"] * (num_waypoints - 2),
    #             name="Intermediate",
    #         ),
    #         go.Scatter(
    #             x=[state.all_local_waypoints_x[-1]],
    #             y=[state.all_local_waypoints_y[-1]],
    #             mode="markers",
    #             marker=go.scatter.Marker(size=14),
    #             text=["Goal"] * (num_waypoints - 2),
    #             name="Goal",
    #         ),
    #     ]
    #     new_frames = [
    #         go.Frame(
    #             data=initial_state
    #             + [
    #                 go.Scatter(
    #                     x=[state.sailbot_pos_x[i]],
    #                     y=[state.sailbot_pos_y[i]],
    #                     mode="markers",
    #                     marker_symbol=self.raw_symbols,
    #                     marker_line_color="darkseagreen",
    #                     marker_color="lightgreen",
    #                     marker_line_width=2,
    #                     marker_size=15,
    #                     text=["Boat"],
    #                     name="Boat",
    #                 )
    #             ],
    #             name=f"Boat {i}",
    #         )
    #         for i in range(0, len(state.sailbot_pos_xy))
    #     ]
    #     fig = go.Figure(
    #         data=initial_state + [initial_boat_state],
    #         layout=go.Layout(
    #             mapbox_style="open-street-map",
    #             mapbox_zoom=7,
    #             mapbox_center={
    #                 "X": state.all_local_waypoints_x[center_index],
    #                 "Y": state.all_local_waypoints_x[center_index],
    #             },
    #             margin={"r": 0, "t": 0, "l": 0, "b": 0},
    #             xaxis=dict(title="X"),
    #             yaxis=dict(title="Y"),
    #             updatemenus=[
    #                 dict(
    #                     type="buttons",
    #                     showactive=False,
    #                     buttons=[
    #                         dict(
    #                             label="Play",
    #                             method="animate",
    #                             args=[
    #                                 None,
    #                                 {
    #                                     "frame": {"duration": 1000, "redraw": True},
    #                                     "mode": "immediate",
    #                                 },
    #                             ],
    #                         ),
    #                         dict(
    #                             label="Pause",
    #                             method="animate",
    #                             args=[
    #                                 [None],
    #                                 {
    #                                     "frame": {"duration": 0, "redraw": True},
    #                                     "mode": "immediate",
    #                                 },
    #                             ],
    #                         ),
    #                     ],
    #                 )
    #             ],
    #         ),
    #         frames=new_frames,
    #     )

    #     fig.show()


def dash_app():

    app.layout = html.Div(
        [
            dcc.Graph(id="live-graph"),
            dcc.Interval(id="interval-component", interval=5000, n_intervals=0),
        ]
    )
    app.title = "Sailbot Path Planning"
    app.run(debug=True, use_reloader=False)


def initial_plot():
    figure = go.Figure()
    fig = go.FigureWidget(figure)

    fig.update_layout(
        title="Path Planning",
        xaxis_title="X Coordinate",
        yaxis_title="Y Coordinate",
        xaxis=dict(range=[-100, 100]),
        yaxis=dict(range=[-100, 100]),
    )

    return fig


@app.callback(Output("live-graph", "figure"), [Input("interval-component", "n_intervals")])
def plot(n_intervals):
    state = queue.get()
    fig = update_plot(state)
    return fig


def update_plot(state: VisualizerState):
    raw_symbols = 'arrow'
    fig = initial_plot()

    # Plotting Local Waypoints
    start_trace = go.Scatter(
        x=[state.last_local_waypoints_x[0]],
        y=[state.last_local_waypoints_y[0]],
        mode="markers",
        marker=dict(color="green", size=10),
        name="Start",
    )

    # Intermediate waypoints (blue)
    intermediate_trace = go.Scatter(
        x=state.last_local_waypoints_x[1:-1],
        y=state.last_local_waypoints_y[1:-1],
        mode="markers",
        marker=dict(color="blue", size=8),
        name="Intermediate",
    )

    # Goal point (red)
    goal_trace = go.Scatter(
        x=[state.last_local_waypoints_x[-1]],
        y=[state.last_local_waypoints_y[-1]],
        mode="markers",
        marker=dict(color="red", size=10),
        name="Goal",
    )

    # Boat marker (current position, red with larger size)
    boat_trace = go.Scatter(
        x=[state.sailbot_pos_x[-1]],
        y=[state.sailbot_pos_y[-1]],
        mode="markers",
        marker_symbol=raw_symbols,
        marker_line_color="darkseagreen",
        marker_color="lightgreen",
        marker_line_width=2,
        marker_size=15,
        name="Boat",
    )

    # Add all traces to the figure
    fig.add_trace(start_trace)
    fig.add_trace(intermediate_trace)
    fig.add_trace(goal_trace)
    fig.add_trace(boat_trace)

    # Set axis limits dynamically
    x_min = min(state.last_local_waypoints_x) - 10
    x_max = max(state.last_local_waypoints_x) + 10
    y_min = min(state.last_local_waypoints_y) - 10
    y_max = max(state.last_local_waypoints_y) + 10

    # Update Layout
    fig.update_layout(
        title="Path Planning",
        xaxis_title="X Coordinate",
        yaxis_title="Y Coordinate",
        xaxis=dict(range=[x_min, x_max]),
        yaxis=dict(range=[y_min, y_max]),
        legend=dict(x=0, y=1),  # Position the legend at the top left
        showlegend=True,
    )

    return fig


def ros_node(args=None):
    rclpy.init(args=args)
    sailbot_observer = SailbotObserver()

    try:
        rclpy.spin(node=sailbot_observer)
    except KeyboardInterrupt:
        sailbot_observer.get_logger().info("Keyboard interrupt [^C], shutting down.")
    finally:

        sailbot_observer.plotting_process.terminate()
        sailbot_observer.plotting_process.join()

        sailbot_observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
