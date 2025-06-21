"""
Sailbot Path Planning Visualizer

This script sets up a Dash web application to visualize the sailbot's path planning data.
It processes ROS messages such as waypoint coordinates and GPS data, converting them from
latitude/longitude to x/y coordinates.

Main Components:
1. VisualizerState: A class that processes the ROS messages and converts coordinates.
2. Dash App: A web application that displays the path planning data in real-time.
3. Plotting Functions: Functions to create and update the plots based on the processed data.
4. Callbacks: Functions that update the plots at regular intervals.

usage:
Call the `dash_app` function with a multiprocessing shared Manager.queue to start the Dash app.
"""

from multiprocessing import Queue
from typing import List, Optional, Tuple

import custom_interfaces.msg as ci
import dash
import plotly.graph_objects as go
from dash import dcc, html
from dash.dependencies import Input, Output
import math
import local_pathfinding.coord_systems as cs

app = dash.Dash(__name__)

queue: Optional[Queue] = None  # type: ignore


class VisualizerState:
    """
    Converts the ROS message to a format that can be used by the visualizer.
    For example, it converts the lat/lon coordinates to x/y coordinates for GPS and Global/Local
    paths.


    Attributes:
        sailbot_lat_lon (ci.LPathData): List of lat/lon coordinates of the sailbot
        all_local_wp (List[List[Tuple[float, float]]]): List of all local waypoints
        global_path (ci.Path): Global path
        reference_latlon (ci.HelperLatLon): Reference lat/lon coordinates
        sailbot_xy (List[Tuple[float, float]]): List of x/y coordinates of the sailbot
        all_wp_xy (List[List[Tuple[float, float]]]): List of all local waypoints in x/y coordinates
        sailbot_pos_x (List[Tuple[float, float]]): X coordinates of the sailbot
        sailbot_pos_y (List[Tuple[float, float]]): Y coordinates of the sailbot
        final_local_wp_x (List[Tuple[float, float]]): X coordinates of the final local waypoint
        final_local_wp_y (List[Tuple[float, float]]): Y coordinates of the final local waypoint
    """

    def __init__(self, msgs: List[ci.LPathData]):
        if not msgs:
            raise ValueError("msgs must not be None")

        self.curr_msg = msgs[-1]
        self._validate_message(self.curr_msg)

        self.sailbot_lat_lon = [msg.gps.lat_lon for msg in msgs]
        self.sailbot_gps = [msg.gps for msg in msgs]
        self.all_local_wp = [msg.local_path.waypoints for msg in msgs]

        self.global_path = self.curr_msg.global_path
        self.reference_latlon = self.global_path.waypoints[-1]

        # Converts the lat/lon coordinates to x/y coordinates
        self.sailbot_xy = cs.latlon_list_to_xy_list(self.reference_latlon, self.sailbot_lat_lon)
        self.all_wp_xy = [
            cs.latlon_list_to_xy_list(self.reference_latlon, waypoints)
            for waypoints in self.all_local_wp
        ]

        # Splits the x/y coordinates into separate lists
        self.sailbot_pos_x, self.sailbot_pos_y = self._split_coordinates(self.sailbot_xy)

        self.final_local_wp_x, self.final_local_wp_y = self._split_coordinates(self.all_wp_xy[-1])
        self.all_local_wp_x, self.all_local_wp_y = zip(
            *[self._split_coordinates(waypoints) for waypoints in self.all_wp_xy]
        )

        # TODO: Include other LPathData attributes for plotting their data

    def _validate_message(self, msg: ci.LPathData):
        """Checks if the sailbot observer node recieved any messages.
        If not, it raises a ValueError.
        """
        if not msg.local_path:
            raise ValueError("local path must not be None")
        if not msg.global_path:
            raise ValueError("global path must not be None")
        if not msg.gps:
            raise ValueError("gps must not be None")

    def _split_coordinates(self, positions) -> Tuple[List[float], List[float]]:
        """Splits a list of positions into x and y components."""
        x_coords = [pos.x for pos in positions]
        y_coords = [pos.y for pos in positions]
        return x_coords, y_coords


def initial_plot() -> go.Figure:
    """
    Initializes the plot with default settings.
    """

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


def dash_app(q: Queue):
    """
    Creates a Dash app and sets up the HTML layout of the app.

    Args:
        q (Queue): The queue to receive data from the ROS node.
    """
    # Allows it to be accessed in the callbacks
    global queue  # type: ignore
    queue = q

    app.layout = html.Div(
        [
            html.H2("Live Path Planning"),
            dcc.Graph(id="live-graph"),
            dcc.Interval(id="interval-component", interval=5000, n_intervals=0),
            html.H2("Animated Path Planning"),
            dcc.Graph(id="animated-live-graph"),
            dcc.Interval(id="interval-component2", interval=10000, n_intervals=0),
            # TODO: Add more graphs and data visualizations as needed
            # dcc.Graph(id="another-graph"),
            # dcc.Interval(id="another-interval", interval=1000, n_intervals=0),
        ],
    )
    app.title = "Sailbot Path Planning"
    app.run(debug=True, use_reloader=False)


@app.callback(Output("live-graph", "figure"), [Input("interval-component", "n_intervals")])
def live_plot(n_intervals) -> go.Figure:
    """
    Updates the live graph to the latest path planning data.
    """
    global queue
    state = queue.get()  # type: ignore
    fig = live_update_plot(state)
    return fig


@app.callback(
    Output("animated-live-graph", "figure"), [Input("interval-component2", "n_intervals")]
)
def animated_plot(n_intervals) -> go.Figure:
    """
    Updates the animated graph to the accumulated LPathData ROS messages.
    """
    global queue
    state = queue.get()  # type: ignore
    fig = animated_update_plot(state)
    return fig


# TODO: Add more callbacks for other graphs and data visualizations as needed


def live_update_plot(state: VisualizerState) -> go.Figure:
    """
    Updates the live graph to the latest path planning data.
    """

    fig = initial_plot()

    # local path waypoints
    intermediate_trace = go.Scatter(
        x=state.final_local_wp_x[1:-1],
        y=state.final_local_wp_y[1:-1],
        mode="markers",
        marker=dict(color="blue", size=8),
        name="Intermediate",
    )

    goal_x = [state.final_local_wp_x[-1]]
    goal_y = [state.final_local_wp_y[-1]]
    boat_x = [state.sailbot_pos_x[-1]]
    boat_y = [state.sailbot_pos_y[-1]]
    angle_from_boat = math.atan2(goal_x[0] - boat_x[0], goal_y[0] - boat_y[0])
    angle_degrees = math.degrees(angle_from_boat)

    # goal local waypoint
    goal_trace = go.Scatter(
        x=goal_x,
        y=goal_y,
        mode="markers",
        marker=dict(color="red", size=10),
        name="Goal",
        hovertemplate="X: %{x:.2f} <br>" +
        "Y: %{y:.2f} <br>" +
        "Angle from the boat: " + f"{angle_degrees:.1f}°" +
        "<extra></extra>"
    )

    # boat marker (current position)
    boat_trace = go.Scatter(
        x=[state.sailbot_pos_x[-1]],
        y=[state.sailbot_pos_y[-1]],
        mode="markers",
        marker_symbol="arrow",
        marker_line_color="darkseagreen",
        marker_color="lightgreen",
        marker_line_width=2,
        marker_size=15,
        name="Boat",
        hovertemplate="<b>🚢 Sailbot Current Position</b><br>" +
        "X: %{x:.2f} <br>" +
        "Y: %{y:.2f} <br>" +
        "Heading: " + f"{state.sailbot_gps[-1].heading.heading:.1f}°<br>" +
        "Speed: " + f"{state.sailbot_gps[-1].speed.speed:.1f}<br>" +
        "<extra></extra>"
    )

    # Add all traces to the figure
    fig.add_trace(intermediate_trace)
    fig.add_trace(goal_trace)
    fig.add_trace(boat_trace)

    # Set axis limits dynamically
    x_min = min(state.final_local_wp_x) - 10
    x_max = max(state.final_local_wp_x) + 10
    y_min = min(state.final_local_wp_y) - 10
    y_max = max(state.final_local_wp_y) + 10

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


def animated_update_plot(state: VisualizerState) -> go.Figure:
    """
    Generates an animated plot every interval with the aggregated LPathData ROS messages.
    It is interactive with play/pause buttons.

    """

    # Initializing the plot
    fig = initial_plot()

    num_waypoints = len(state.all_wp_xy[-1])
    initial_boat_state = go.Scatter(
        x=[state.sailbot_pos_x[0]],
        y=[state.sailbot_pos_y[0]],
        mode="markers",
        marker_symbol="arrow",
        marker_line_color="darkseagreen",
        marker_color="lightgreen",
        marker_line_width=2,
        marker_size=15,
        text=["Boat"],
        name="Boat",
        hovertemplate="<b>🚢 Sailbot Current Position</b><br>" +
        "X: %{x:.2f} meters<br>" +
        "Y: %{y:.2f} meters<br>" +
        "Heading: " + f"{state.sailbot_gps[0].heading.heading:.1f}°<br>" +
        "Speed: " + f"{state.sailbot_gps[0].speed.speed:.1f}<br>" +
        "<extra></extra>"
    )
    initial_state = [
        go.Scatter(
            x=[state.all_local_wp_x[0][0]],
            y=[state.all_local_wp_y[0][0]],
            mode="markers",
            marker=go.scatter.Marker(size=14),
            text=["Start"],
            name="Start",
        ),
        go.Scatter(
            x=state.all_local_wp_x[0][1 : num_waypoints - 1],
            y=state.all_local_wp_y[0][1 : num_waypoints - 1],
            mode="markers",
            marker=go.scatter.Marker(size=14),
            text=["Intermediate"] * (num_waypoints - 2),
            name="Intermediate",
        ),
        go.Scatter(
            x=[state.all_local_wp_x[0][-1]],
            y=[state.all_local_wp_y[0][-1]],
            mode="markers",
            marker=go.scatter.Marker(size=14),
            text=["Goal"] * (num_waypoints - 2),
            name="Goal",
        ),
    ]
    new_frames = [
        go.Frame(
            data=[
                go.Scatter(
                    x=[state.all_local_wp_x[i][0]],
                    y=[state.all_local_wp_y[i][0]],
                    mode="markers",
                    marker=go.scatter.Marker(size=14),
                    text=["Start"],
                    name="Start",
                ),
                go.Scatter(
                    x=state.all_local_wp_x[i][1 : num_waypoints - 1],
                    y=state.all_local_wp_y[i][1 : num_waypoints - 1],
                    mode="markers",
                    marker=go.scatter.Marker(size=14),
                    text=["Intermediate"] * (num_waypoints - 2),
                    name="Intermediate",
                ),
                go.Scatter(
                    x=[state.all_local_wp_x[i][-1]],
                    y=[state.all_local_wp_y[i][-1]],
                    mode="markers",
                    marker=go.scatter.Marker(size=14),
                    text=["Goal"] * (num_waypoints - 2),
                    name="Goal",
                ),
            ]
            + [
                go.Scatter(
                    x=[state.sailbot_pos_x[i]],
                    y=[state.sailbot_pos_y[i]],
                    mode="markers",
                    marker_symbol="arrow",
                    marker_line_color="darkseagreen",
                    marker_color="lightgreen",
                    marker_line_width=2,
                    marker_size=15,
                    text=["Boat"],
                    name="Boat",
                    hovertemplate="<b>🚢 Sailbot Current Position</b><br>" +
                    "X: %{x:.2f} meters<br>" +
                    "Y: %{y:.2f} meters<br>" +
                    "Heading: " + f"{state.sailbot_gps[i].heading.heading:.1f}°<br>" +
                    "Speed: " + f"{state.sailbot_gps[i].speed.speed:.1f}<br>" +
                    "<extra></extra>"
                )
            ],
            name=f"Boat {i}",
        )
        for i in range(0, len(state.sailbot_xy))
    ]

    # Set axis limits dynamically
    x_min = min(state.final_local_wp_x) - 10
    x_max = max(state.final_local_wp_x) + 10
    y_min = min(state.final_local_wp_y) - 10
    y_max = max(state.final_local_wp_y) + 10

    # Set up the animated plot
    fig = go.Figure(
        data=initial_state + [initial_boat_state],
        layout=go.Layout(
            xaxis_title="X Coordinate",
            yaxis_title="Y Coordinate",
            xaxis=dict(range=[x_min, x_max]),
            yaxis=dict(range=[y_min, y_max]),
            legend=dict(x=0, y=1),  # Position the legend at the top left
            showlegend=True,
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
                                    "fromcurrent": True,
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
                                    "fromcurrent": False,
                                },
                            ],
                        ),
                    ],
                )
            ],
        ),
        frames=new_frames,
    )

    return fig


# TODO: Add more plotting functions as needed
