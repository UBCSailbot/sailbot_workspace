from multiprocessing import Queue
from typing import List, Optional

import custom_interfaces.msg as ci
import dash
import plotly.graph_objects as go
from dash import dcc, html
from dash.dependencies import Input, Output

from local_pathfinding.coord_systems import latlon_to_xy

app = dash.Dash(__name__)

queue: Optional[Queue] = None  # type: ignore


class VisualizerState:
    """
    Converts the ROS message to a format that can be used by the visualizer.
    """

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
        ]

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
            [point.x for point in waypoints] for waypoints in self.all_waypoints_pos_xy
        ]

        self.all_local_waypoints_y = [
            [point.y for point in waypoints] for waypoints in self.all_waypoints_pos_xy
        ]


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


def live_update_plot(state: VisualizerState) -> go.Figure:
    """
    Updates the live graph to the latest path planning data.
    """

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
        marker_symbol="arrow",
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


def animated_update_plot(state: VisualizerState) -> go.Figure:
    """
    Generates an animated plot every interval with the aggregated LPathData ROS messages.
    It is interactive with play/pause buttons.

    """

    # Initializing the plot
    fig = initial_plot()

    num_waypoints = len(state.all_waypoints_pos_xy[-1])
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
    )
    initial_state = [
        go.Scatter(
            x=[state.all_local_waypoints_x[0][0]],
            y=[state.all_local_waypoints_y[0][0]],
            mode="markers",
            marker=go.scatter.Marker(size=14),
            text=["Start"],
            name="Start",
        ),
        go.Scatter(
            x=state.all_local_waypoints_x[0][1 : num_waypoints - 1],
            y=state.all_local_waypoints_y[0][1 : num_waypoints - 1],
            mode="markers",
            marker=go.scatter.Marker(size=14),
            text=["Intermediate"] * (num_waypoints - 2),
            name="Intermediate",
        ),
        go.Scatter(
            x=[state.all_local_waypoints_x[0][-1]],
            y=[state.all_local_waypoints_y[0][-1]],
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
                    x=[state.all_local_waypoints_x[i][0]],
                    y=[state.all_local_waypoints_y[i][0]],
                    mode="markers",
                    marker=go.scatter.Marker(size=14),
                    text=["Start"],
                    name="Start",
                ),
                go.Scatter(
                    x=state.all_local_waypoints_x[i][1 : num_waypoints - 1],
                    y=state.all_local_waypoints_y[i][1 : num_waypoints - 1],
                    mode="markers",
                    marker=go.scatter.Marker(size=14),
                    text=["Intermediate"] * (num_waypoints - 2),
                    name="Intermediate",
                ),
                go.Scatter(
                    x=[state.all_local_waypoints_x[i][-1]],
                    y=[state.all_local_waypoints_y[i][-1]],
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
                )
            ],
            name=f"Boat {i}",
        )
        for i in range(0, len(state.sailbot_pos_xy))
    ]

    # Set axis limits dynamically
    x_min = min(state.last_local_waypoints_x) - 10
    x_max = max(state.last_local_waypoints_x) + 10
    y_min = min(state.last_local_waypoints_y) - 10
    y_max = max(state.last_local_waypoints_y) + 10

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
