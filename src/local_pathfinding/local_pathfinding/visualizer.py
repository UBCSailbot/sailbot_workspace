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

import math
from multiprocessing import Queue
from typing import List, Optional, Tuple

import custom_interfaces.msg as ci
import dash
import plotly.graph_objects as go
from dash import dcc, html
from dash.dependencies import Input, Output
from shapely.geometry import MultiPolygon, Polygon

import local_pathfinding.coord_systems as cs
from local_pathfinding.ompl_objectives import get_true_wind
from local_pathfinding.ompl_path import OMPLPath


app = dash.Dash(__name__)

queue: Optional[Queue] = None  # type: ignore

BOX_BUFFER_SIZE = 1.0  # km

LAST_GOAL = None  # for the msg_to_display


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

        # AIS ships
        self.ais_ships = self.curr_msg.ais_ships.ships
        ais_ship_latlons = [ship.lat_lon for ship in self.ais_ships]
        self.ais_ship_ids = [ship.id for ship in self.ais_ships]
        ais_ship_xy = cs.latlon_list_to_xy_list(self.reference_latlon, ais_ship_latlons)
        self.ais_pos_x, self.ais_pos_y = self._split_coordinates(ais_ship_xy)
        self.ais_headings = [ship.cog.heading for ship in self.ais_ships]

        # TODO: Include other LPathData attributes for plotting their data

        # Process land obstacles
        self.land_obstacles_xy = self._process_land_obstacles(
            self.curr_msg.obstacles, self.reference_latlon
        )

        # Process Boat Obstacles
        self.boat_obstacles_xy = self._process_boat_obstacles(
            self.curr_msg.obstacles, self.reference_latlon
            )

        # Process wind vectors

        # apparent wind vector
        self.wind_vector = self._process_apparent_wind_vector(self.curr_msg.filtered_wind_sensor)

        # true wind vector
        boat_sog = self.curr_msg.gps.speed.speed
        boat_heading = self.curr_msg.gps.heading.heading
        true_wind = get_true_wind(
            self.curr_msg.filtered_wind_sensor.direction,
            self.curr_msg.filtered_wind_sensor.speed.speed,
            boat_heading,
            boat_sog,
        )
        self.true_wind_vector = self._process_true_wind_vector(true_wind)

        # boat wind vector
        # The boat's motion creates an apparent wind in the opposite direction of its heading,
        # so we add 180°
        boat_wind_radians = math.radians(cs.bound_to_180(boat_heading + 180))
        boat_wind_east = boat_sog * math.sin(boat_wind_radians)
        boat_wind_north = boat_sog * math.cos(boat_wind_radians)
        self.boat_wind_vector = cs.XY(boat_wind_east, boat_wind_north)

    def _validate_message(self, msg: ci.LPathData):
        """Checks if the sailbot observer node received any messages.
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

    def _process_apparent_wind_vector(self, wind_sensor):
        """
        Processes wind_sensor data to extract the apparent wind vector components
        """
        speed = wind_sensor.speed.speed
        direction_deg = wind_sensor.direction
        direction_rad = math.radians(direction_deg)

        dx = speed * math.sin(direction_rad)
        dy = speed * math.cos(direction_rad)

        return cs.XY(x=dx, y=dy)

    def _process_true_wind_vector(self, true_wind):
        true_wind_direction_rad, true_wind_magnitude = true_wind
        dx = true_wind_magnitude * math.sin(true_wind_direction_rad)
        dy = true_wind_magnitude * math.cos(true_wind_direction_rad)

        return cs.XY(x=dx, y=dy)

    def _process_land_obstacles(self, obstacles, reference):
        """
        Converts land obstacles from latitude/longitude to XY coordinates and builds Shapely
        polygons.
        """
        processed_obstacles = []

        for ob in obstacles:
            if ob.obstacle_type == "Land":
                # Convert each latlon point to XY
                xy_points = []
                for point in ob.points:
                    xy = cs.latlon_to_xy(reference, point)
                    xy_points.append((xy.x, xy.y))

                if len(xy_points) >= 3:
                    poly = Polygon(xy_points)
                    processed_obstacles.append(poly)

        return processed_obstacles

    def _process_boat_obstacles(self, obstacles, reference):
        """
        Converts Boat obstacles from latitude/longitude to XY coordinates and builds Shapely
        polygons.
        processed_obstacles of type List[Polygon].
        """
        processed_obstacles = []
        if obstacles is None:
            return processed_obstacles

        for obs in obstacles:
            if obs.obstacle_type != "Boat":
                continue
            # Convert each latlon point to XY
            xy_points = []
            for point in obs.points:
                xy = cs.latlon_to_xy(reference, point)
                xy_points.append((xy.x, xy.y))

            if len(xy_points) >= 3:
                poly = Polygon(xy_points)
                processed_obstacles.append(poly)

        return processed_obstacles


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

    # app.layout = html.Div(
    #     [
    #         html.H2("Live Path Planning"),
    #         dcc.Graph(id="live-graph"),
    #         dcc.Interval(id="interval-component", interval=5000, n_intervals=0),
    #         # Commented out animated path planning
    #         # html.H2("Animated Path Planning"),
    #         # dcc.Graph(id="animated-live-graph"),
    #         # dcc.Interval(id="interval-component2", interval=10000, n_intervals=0),
    #         # TODO: Add more graphs and data visualizations as needed
    #         # dcc.Graph(id="another-graph"),
    #         # dcc.Interval(id="another-interval", interval=1000, n_intervals=0),
    #     ],
    # )
    # app.title = "Sailbot Path Planning"
    # app.run(debug=True, use_reloader=False)

    app.layout = html.Div(
        style={"height": "100vh", "width": "100vw", "margin": 0, "padding": 0},
        children=[
            html.H2("Live Path Planning"),
            dcc.Graph(
                id="live-graph",
                style={"height": "90vh", "width": "100%"}
            ),
            dcc.Interval(
                id="interval-component",
                interval=5000,
                n_intervals=0
            ),
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


# Commented out animated path planning
# @app.callback(
#     Output("animated-live-graph", "figure"), [Input("interval-component2", "n_intervals")]
# )
# def animated_plot(n_intervals) -> go.Figure:
#     """
#     Updates the animated graph to the accumulated LPathData ROS messages.
#     """
#     global queue
#     state = queue.get()  # type: ignore
#     fig = animated_update_plot(state)
#     return fig


# TODO: Add more callbacks for other graphs and data visualizations as needed


def live_update_plot(state: VisualizerState) -> go.Figure:
    """
    Updates the live graph to the latest path planning data.
    """

    fig = initial_plot()

    # local path waypoints
    ix = state.final_local_wp_x
    iy = state.final_local_wp_y
    labels = [f"LW{i}" for i in range(1, max(1, len(ix)-1))]
    intermediate_trace = go.Scatter(
        x=ix[1:-1],
        y=iy[1:-1],
        mode="markers+text",
        marker=dict(color="blue", size=8),
        text=labels,
        textposition="top center",
        name="Intermediate",
    )

    goal_x = [state.final_local_wp_x[-1]]
    goal_y = [state.final_local_wp_y[-1]]
    boat_x = [state.sailbot_pos_x[-1]]
    boat_y = [state.sailbot_pos_y[-1]]
    angle_from_boat = math.atan2(goal_x[0] - boat_x[0], goal_y[0] - boat_y[0])
    angle_degrees = math.degrees(angle_from_boat)

    global LAST_GOAL
    msg_to_display = None
    gh = (round(goal_x[0], 3), round(goal_y[0], 3))
    if LAST_GOAL is not None and gh != LAST_GOAL:
        msg_to_display = f"Local goal advanced to ({gh[0]}, {gh[1]})"
    LAST_GOAL = gh

    # goal local waypoint
    goal_trace = go.Scatter(
        x=goal_x,
        y=goal_y,
        mode="markers",
        marker=dict(color="red", size=10),
        name="Goal",
        hovertemplate="X: %{x:.2f} <br>"
        + "Y: %{y:.2f} <br>"
        + "Angle from the boat: "
        + f"{angle_degrees:.1f}°"
        + "<extra></extra>",
    )

    boat_trace = go.Scatter(
        x=[state.sailbot_pos_x[-1]],
        y=[state.sailbot_pos_y[-1]],
        mode="markers",
        name="Boat",
        hovertemplate=(
            "<b>🚢 Sailbot Current Position</b><br>"
            "X: %{x:.2f} <br>"
            "Y: %{y:.2f} <br>"
            "Heading: " + f"{state.sailbot_gps[-1].heading.heading:.1f}°<br>"
            f"Speed: {state.sailbot_gps[-1].speed.speed:.1f}<br>"
            "<extra></extra>"
        ),
        marker=dict(
            symbol="arrow-wide",
            line_color="darkseagreen",
            color="lightgreen",
            line_width=2,
            size=15,
            angleref="up",
            angle=cs.true_bearing_to_plotly_cartesian(state.sailbot_gps[-1].heading.heading),
        ),
    )

    # land obstacles
    for poly in state.land_obstacles_xy:
        if not poly.is_empty:
            x = list(poly.exterior.xy[0])
            y = list(poly.exterior.xy[1])
            fig.add_trace(
                go.Scatter(
                    x=x,
                    y=y,
                    fill="toself",
                    mode="lines",
                    line=dict(color="lightgreen"),
                    fillcolor="lightgreen",
                    opacity=0.5,
                    name="Land Obstacle",
                )
            )

    # box for boat wind vector and true wind vector
    fig.update_layout(
        xaxis2=dict(
            domain=[0.85, 0.98],
            anchor="y2",
            range=[-10, 10],
            showgrid=False,
            zeroline=True,
            visible=False,
        ),
        yaxis2=dict(
            domain=[0.05, 0.25],
            anchor="x2",
            range=[-10, 10],
            showgrid=False,
            zeroline=True,
            visible=False,
        ),
    )

    # apparent wind vector in box
    dx = state.wind_vector.x * 1.5
    dy = state.wind_vector.y * 1.5
    fig.add_annotation(
        x=4,
        y=0,
        ax=4 - dx,
        ay=0 - dy,
        xref="x2",
        yref="y2",
        axref="x2",
        ayref="y2",
        showarrow=True,
        arrowhead=3,
        arrowsize=0.5,
        arrowwidth=3,
        arrowcolor="purple",
        standoff=2,
        text="",
        hovertext=(
            f"<b>🌬️ Apparent Wind</b><br>"
            f"speed: {math.hypot(state.wind_vector.x, state.wind_vector.y):.2f} km/h<br>"
        ),
        hoverlabel=dict(bgcolor="white"),
    )

    # true wind vector in box
    dx = state.true_wind_vector.x * 1.5
    dy = state.true_wind_vector.y * 1.5
    fig.add_annotation(
        x=4,
        y=0,
        ax=4 - dx,
        ay=0 - dy,
        xref="x2",
        yref="y2",
        axref="x2",
        ayref="y2",
        showarrow=True,
        arrowhead=3,
        arrowsize=0.5,
        arrowwidth=3,
        arrowcolor="blue",
        standoff=2,
        text="",
        hovertext=(
            f"<b>🌬️ True Wind</b><br>"
            f"speed: {math.hypot(state.true_wind_vector.x, state.true_wind_vector.y):.2f} km/h<br>"
        ),
        hoverlabel=dict(bgcolor="white"),
    )

    # boat vector in box
    dx = state.boat_wind_vector.x * 1.5
    dy = state.boat_wind_vector.y * 1.5
    fig.add_annotation(
        x=4,
        y=0,
        ax=4 - dx,
        ay=0 - dy,
        xref="x2",
        yref="y2",
        axref="x2",
        ayref="y2",
        showarrow=True,
        arrowhead=3,
        arrowsize=0.5,
        arrowwidth=3,
        arrowcolor="red",
        standoff=2,
        text="",
        hovertext=(
            f"<b>🛶 Boat Wind</b><br>"
            f"speed: {math.hypot(state.boat_wind_vector.x, state.boat_wind_vector.y):.2f} km/h<br>"
        ),
        hoverlabel=dict(bgcolor="white"),
    )

    # Fill in box
    fig.add_shape(
        type="rect",
        xref="paper",
        yref="paper",
        x0=0.85,
        y0=0.05,
        x1=0.98,
        y1=0.25,
        fillcolor="white",
        line=dict(width=4),
    )

    # add boat and true wind labels

    fig.add_annotation(
        x=-8,
        y=4,
        xref="x2",
        yref="y2",
        text="Boat",
        showarrow=False,
        align="left",
        xanchor="left",
        font=dict(size=12, color="red"),
    )

    fig.add_annotation(
        x=-8,
        y=0,
        xref="x2",
        yref="y2",
        text="True",
        showarrow=False,
        align="left",
        xanchor="left",
        font=dict(size=12, color="blue"),
    )

    fig.add_annotation(
        x=-8,
        y=-4,
        xref="x2",
        yref="y2",
        text="Apparent",
        showarrow=False,
        align="left",
        xanchor="left",
        font=dict(size=12, color="purple"),
    )

    # Box title
    fig.add_annotation(
        x=0,
        y=6,
        xref="x2",
        yref="y2",
        showarrow=False,
        text="<b>Wind</b>",
        font=dict(size=12, color="black"),
    )

    # Circle representing boat on wind box
    fig.add_annotation(
        x=4,
        y=0,
        xref="x2",
        yref="y2",
        showarrow=False,
        text="●",  # Unicode solid circle
        font=dict(size=12, color="lightgreen"),
    )

    # Draw Boat State space
    boat_pos = cs.XY(boat_x[0], boat_y[0])
    goal_pos = cs.XY(goal_x[0], goal_y[0])

    boat_box = OMPLPath.create_buffer_around_position(boat_pos, BOX_BUFFER_SIZE)
    goal_box = OMPLPath.create_buffer_around_position(goal_pos, BOX_BUFFER_SIZE)

    # Set state space bounds
    state_space = MultiPolygon([boat_box, goal_box])
    x_min, y_min, x_max, y_max = state_space.bounds

    fig.add_shape(
        type="rect",
        x0=x_min, y0=y_min, x1=x_max, y1=y_max,
        fillcolor="rgba(255, 100, 100, 0.25)",  # light red, semi-transparent
        line=dict(width=0),
        layer="below",
    )

    # Path trace (path to goal point)
    if state.final_local_wp_x and state.final_local_wp_y:
        planned_x = [boat_x[0]] + list(state.final_local_wp_x)
        planned_y = [boat_y[0]] + list(state.final_local_wp_y)
        path_trace = go.Scatter(
            x=planned_x,
            y=planned_y,
            mode="lines",
            name="Path to Goal",
            line=dict(width=2, dash="dot", color="orange"),
            hovertemplate="X: %{x:.2f}<br>Y: %{y:.2f}<extra></extra>",
        )

    # Add all traces to the figure
    fig.add_trace(intermediate_trace)
    fig.add_trace(goal_trace)
    fig.add_trace(boat_trace)
    fig.add_trace(path_trace)

    # Set axis limits dynamically
    PAD = 10.0
    x_candidates = [boat_x[0]] + list(state.final_local_wp_x)
    y_candidates = [boat_y[0]] + list(state.final_local_wp_y)
    x_min = min(x_candidates) - PAD
    x_max = max(x_candidates) + PAD
    y_min = min(y_candidates) - PAD
    y_max = max(y_candidates) + PAD

    # Display AIS Ships
    for x_val, y_val, heading, ais_id in zip(
        state.ais_pos_x, state.ais_pos_y, state.ais_headings, state.ais_ship_ids
    ):
        fig.add_trace(
            go.Scatter(
                x=[x_val],
                y=[y_val],
                mode="markers",
                name=f"AIS {str(ais_id)}",
                hovertemplate=(
                    f"<b>🚢 AIS Ship {str(ais_id)}</b><br>"
                    f"X: {x_val:.2f}<br>"
                    f"Y: {y_val:.2f}<br>"
                    f"Heading: {heading:.1f}°<extra></extra>"
                ),
                marker=dict(
                    symbol="arrow-wide",
                    line_color="orange",
                    color="orange",
                    line_width=2,
                    size=15,
                    angleref="up",
                    angle=cs.true_bearing_to_plotly_cartesian(heading),
                ),
                showlegend=True,
            )
        )
    # Display collision zone for Boat Obstacles
    for poly in state.boat_obstacles_xy:
        if poly and not poly.is_empty:
            x = list(poly.exterior.xy[0])
            y = list(poly.exterior.xy[1])
            fig.add_trace(
                go.Scatter(
                    x=x,
                    y=y,
                    mode="lines",
                    fill="toself",
                    line=dict(width=2),
                    fillcolor="rgba(255,165,0,0.25)",
                    name="AIS Collision Zone",
                    hoverinfo="skip",
                    showlegend=True,
                    opacity=0.5,
                )
            )

    if msg_to_display:
        fig.add_annotation(
            text=msg_to_display,
            xref="paper", yref="paper",
            x=0.02, y=0.98,
            showarrow=False,
            bgcolor="rgba(255,230,150,0.9)",
            bordercolor="rgba(0,0,0,0.2)",
            borderwidth=1,
        )

    # Update Layout
    fig.update_layout(
        title="Path Planning",
        xaxis_title="X Coordinate",
        yaxis_title="Y Coordinate",
        xaxis=dict(range=[x_min, x_max]),
        yaxis=dict(range=[y_min, y_max]),
        legend=dict(x=0, y=1),  # Position the legend at the top left
        showlegend=True,
        uirevision="stay"
    )

    return fig


# def animated_update_plot(state: VisualizerState) -> go.Figure:
#     """
#     Generates an animated plot every interval with the aggregated LPathData ROS messages.
#     It is interactive with play/pause buttons.

#     """

#     # Initializing the plot
#     fig = initial_plot()

#     num_waypoints = len(state.all_wp_xy[-1])
#     initial_boat_state = go.Scatter(
#         x=[state.sailbot_pos_x[0]],
#         y=[state.sailbot_pos_y[0]],
#         mode="markers",
#         marker_symbol="arrow",
#         marker_line_color="darkseagreen",
#         marker_color="lightgreen",
#         marker_line_width=2,
#         marker_size=15,
#         text=["Boat"],
#         name="Boat",
#         hovertemplate="<b>🚢 Sailbot Current Position</b><br>"
#         + "X: %{x:.2f} meters<br>"
#         + "Y: %{y:.2f} meters<br>"
#         + "Heading: "
#         + f"{state.sailbot_gps[0].heading.heading:.1f}°<br>"
#         + "Speed: "
#         + f"{state.sailbot_gps[0].speed.speed:.1f}<br>"
#         + "<extra></extra>",
#     )
#     initial_state = [
#         go.Scatter(
#             x=[state.all_local_wp_x[0][0]],
#             y=[state.all_local_wp_y[0][0]],
#             mode="markers",
#             marker=go.scatter.Marker(size=14),
#             text=["Start"],
#             name="Start",
#         ),
#         go.Scatter(
#             x=state.all_local_wp_x[0][1 : num_waypoints - 1],
#             y=state.all_local_wp_y[0][1 : num_waypoints - 1],
#             mode="markers",
#             marker=go.scatter.Marker(size=14),
#             text=["Intermediate"] * (num_waypoints - 2),
#             name="Intermediate",
#         ),
#         go.Scatter(
#             x=[state.all_local_wp_x[0][-1]],
#             y=[state.all_local_wp_y[0][-1]],
#             mode="markers",
#             marker=go.scatter.Marker(size=14),
#             text=["Goal"] * (num_waypoints - 2),
#             name="Goal",
#         ),
#     ]
#     new_frames = [
#         go.Frame(
#             data=[
#                 go.Scatter(
#                     x=[state.all_local_wp_x[i][0]],
#                     y=[state.all_local_wp_y[i][0]],
#                     mode="markers",
#                     marker=go.scatter.Marker(size=14),
#                     text=["Start"],
#                     name="Start",
#                 ),
#                 go.Scatter(
#                     x=state.all_local_wp_x[i][1 : num_waypoints - 1],
#                     y=state.all_local_wp_y[i][1 : num_waypoints - 1],
#                     mode="markers",
#                     marker=go.scatter.Marker(size=14),
#                     text=["Intermediate"] * (num_waypoints - 2),
#                     name="Intermediate",
#                 ),
#                 go.Scatter(
#                     x=[state.all_local_wp_x[i][-1]],
#                     y=[state.all_local_wp_y[i][-1]],
#                     mode="markers",
#                     marker=go.scatter.Marker(size=14),
#                     text=["Goal"] * (num_waypoints - 2),
#                     name="Goal",
#                 ),
#             ]
#             + [
#                 go.Scatter(
#                     x=[state.sailbot_pos_x[i]],
#                     y=[state.sailbot_pos_y[i]],
#                     mode="markers",
#                     marker_symbol="arrow",
#                     marker_line_color="darkseagreen",
#                     marker_color="lightgreen",
#                     marker_line_width=2,
#                     marker_size=15,
#                     text=["Boat"],
#                     name="Boat",
#                     hovertemplate="<b>🚢 Sailbot Current Position</b><br>"
#                     + "X: %{x:.2f} meters<br>"
#                     + "Y: %{y:.2f} meters<br>"
#                     + "Heading: "
#                     + f"{state.sailbot_gps[i].heading.heading:.1f}°<br>"
#                     + "Speed: "
#                     + f"{state.sailbot_gps[i].speed.speed:.1f}<br>"
#                     + "<extra></extra>",
#                 )
#             ],
#             name=f"Boat {i}",
#         )
#         for i in range(0, len(state.sailbot_xy))
#     ]

#     # Set axis limits dynamically
#     x_min = min(state.final_local_wp_x) - 10
#     x_max = max(state.final_local_wp_x) + 10
#     y_min = min(state.final_local_wp_y) - 10
#     y_max = max(state.final_local_wp_y) + 10

#     # Set up the animated plot
#     fig = go.Figure(
#         data=initial_state + [initial_boat_state],
#         layout=go.Layout(
#             xaxis_title="X Coordinate",
#             yaxis_title="Y Coordinate",
#             xaxis=dict(range=[x_min, x_max]),
#             yaxis=dict(range=[y_min, y_max]),
#             legend=dict(x=0, y=1),  # Position the legend at the top left
#             showlegend=True,
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
#                                     "fromcurrent": True,
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
#                                     "fromcurrent": False,
#                                 },
#                             ],
#                         ),
#                     ],
#                 )
#             ],
#         ),
#         frames=new_frames,
#     )

#     return fig


# TODO: Add more plotting functions as needed
