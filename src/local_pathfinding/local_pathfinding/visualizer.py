"""
Sailbot Path Planning Visualizer

This script sets up a Dash web application to visualize the sailbot's path planning data.
It processes ROS messages such as waypoint coordinates and GPS data, converting them from
latitude/longitude to XY coordinates in km.

Main Components:
1. VisualizerState: A class that processes the ROS messages and converts coordinates.
2. Dash App: A web application that displays the path planning data in real-time.
3. Plotting Functions: Functions to create and update the plots based on the processed data.
4. Callbacks: Functions that update the plots at regular intervals.
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
import local_pathfinding.wind_coord_systems as wcs
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
        self.sailbot_xy = cs.latlon_list_to_xy_list(self.reference_latlon, self.sailbot_lat_lon)
        self.all_wp_xy = [
            cs.latlon_list_to_xy_list(self.reference_latlon, waypoints)
            for waypoints in self.all_local_wp
        ]
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
        self.ais_speeds = [ship.sog.speed for ship in self.ais_ships]

        # Process land obstacles
        self.land_obstacles_xy = self._process_land_obstacles(
            self.curr_msg.obstacles, self.reference_latlon
        )

        # Process Boat Obstacles
        self.boat_obstacles_xy = self._process_boat_obstacles(
            self.curr_msg.obstacles, self.reference_latlon
        )

        boat_speed = self.curr_msg.gps.speed.speed
        boat_heading = self.curr_msg.gps.heading.heading
        aw_speed = self.curr_msg.filtered_wind_sensor.speed.speed
        aw_dir_boat = self.curr_msg.filtered_wind_sensor.direction

        # Convert Apparent wind to global frame
        aw_dir_global = wcs.boat_to_global_coordinate(boat_heading, aw_dir_boat)
        aw_dir_global_rad = math.radians(aw_dir_global)

        # Compute apparent wind vector (in global frame)
        self.aw_wind_vector = cs.angle_to_vector_projections(aw_dir_global_rad, aw_speed)

        # True wind from apparent
        true_wind_angle_rad, true_wind_mag = wcs.get_true_wind(
            aw_dir_global, aw_speed, boat_heading, boat_speed
        )
        self.true_wind_vector = cs.angle_to_vector_projections(true_wind_angle_rad, true_wind_mag)
        # Boat wind vector
        boat_wind_radians = math.radians(cs.bound_to_180(boat_heading + 180))
        self.boat_wind_vector = cs.angle_to_vector_projections(boat_wind_radians, boat_speed)

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
        """Splits a list of XY objects into their separate x and y components."""
        x_coords = [pos.x for pos in positions]
        y_coords = [pos.y for pos in positions]
        return x_coords, y_coords

    def _process_land_obstacles(self, obstacles, reference):
        """
        Converts land obstacles from latitude/longitude to XY coordinates and builds a Shapely
        polygon for each polygon in the land obstacle's collison zone.
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


def get_unit_vector(vec: cs.XY) -> cs.XY:
    """
    Returns the unit vector of the given vector.
    """
    mag = math.hypot(vec.x, vec.y)
    if mag > 1e-6:
        return cs.XY(vec.x / mag, vec.y / mag)
    return cs.XY(0.0, 0.0)


def initial_plot() -> go.Figure:
    """
    Initializes the plot with default settings.
    """

    figure = go.Figure()
    fig = go.FigureWidget(figure)

    fig.update_layout(
        xaxis_title="X (Km)",
        yaxis_title="Y (Km)",
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
        style={
            "height": "100vh",
            "width": "100vw",
            "margin": 0,
            "padding": 0,
        },
        children=[
            html.H2(
                "UBC Sailbot Pathfinding",
                style={"fontFamily": "Consolas, monospace", "color": "rgb(18, 70, 139)"},
            ),
            dcc.Graph(id="live-graph", style={"height": "90vh", "width": "100%"}),
            dcc.Interval(id="interval-component", interval=2500, n_intervals=0),
        ],
    )
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


def live_update_plot(state: VisualizerState) -> go.Figure:
    """
    Updates the live graph to the latest path planning data.
    """

    fig = initial_plot()

    # local path waypoints
    ix = state.final_local_wp_x
    iy = state.final_local_wp_y
    # iterating from the 2nd to the 2nd last element in ix
    labels = [f"LW{i+1}" for i, _ in enumerate(ix[1:-1])]
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
    distance_to_goal = math.hypot(goal_x[0] - boat_x[0], goal_y[0] - boat_y[0])

    # Track changes in the local goal point to display a popup message when it updates
    global LAST_GOAL
    msg_to_display = None
    # To avoid spam messages due to tiny float jitter
    current_goal_xy = (round(goal_x[0], 3), round(goal_y[0], 3))
    # form the message to display if the local goal point has changed from last time
    if LAST_GOAL is not None and current_goal_xy != LAST_GOAL:
        msg_to_display = f"Local goal advanced to ({current_goal_xy[0]}, {current_goal_xy[1]})"
    LAST_GOAL = current_goal_xy

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

    # Sailbot marker
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
            f"Speed: {state.sailbot_gps[-1].speed.speed:.1f} km/h <br>"
            f"Distance to Goal: {distance_to_goal:.2f} km<br>"
            "<extra></extra>"
        ),
        marker=dict(
            symbol="arrow",
            color="yellow",
            line=dict(width=2, color="DarkSlateGrey"),
            size=20,
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

    # box for boat wind, true wind and apparent wind vectors
    fig.update_layout(
        xaxis2=dict(
            domain=[0.76, 0.99],
            anchor="y2",
            range=[-10, 10],
            showgrid=False,
            zeroline=True,
            visible=False,
        ),
        yaxis2=dict(
            domain=[0.00, 0.22],
            anchor="x2",
            range=[-10, 10],
            showgrid=False,
            zeroline=True,
            visible=False,
        ),
    )

    # Scaling and offset for better visualization
    # Compute magnitudes and scale factor
    aw_unit_vec = get_unit_vector(state.aw_wind_vector)
    tw_unit_vec = get_unit_vector(state.true_wind_vector)
    bw_unit_vec = get_unit_vector(state.boat_wind_vector)

    aw_mag = math.hypot(state.aw_wind_vector.x, state.aw_wind_vector.y)
    tw_mag = math.hypot(state.true_wind_vector.x, state.true_wind_vector.y)
    bw_mag = math.hypot(state.boat_wind_vector.x, state.boat_wind_vector.y)

    ARROW_LEN = 4.0

    # Scaled component vectors
    aw_dx, aw_dy = aw_unit_vec.x * ARROW_LEN, aw_unit_vec.y * ARROW_LEN
    tw_dx, tw_dy = tw_unit_vec.x * ARROW_LEN, tw_unit_vec.y * ARROW_LEN
    bw_dx, bw_dy = bw_unit_vec.x * ARROW_LEN, bw_unit_vec.y * ARROW_LEN

    # Small vertical offsets so the arrows don't overlap
    ORIGIN_X, ORIGIN_Y = 6, 0
    Y_OFFSET = 1.0

    boat_arrow_origin = (ORIGIN_X, ORIGIN_Y + Y_OFFSET)
    true_wind_arrow_origin = (ORIGIN_X, ORIGIN_Y)
    apparent_wind_arrow_origin = (ORIGIN_X, ORIGIN_Y - Y_OFFSET)

    # apparent wind vector in box
    fig.add_annotation(
        x=apparent_wind_arrow_origin[0],
        y=apparent_wind_arrow_origin[1],
        ax=apparent_wind_arrow_origin[0] + aw_dx,
        ay=apparent_wind_arrow_origin[1] + aw_dy,
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
        hovertext=(f"<b>🌬️ Apparent Wind</b><br>" f"speed: {aw_mag:.2f} kmph<br>"),
        hoverlabel=dict(bgcolor="white"),
    )

    # true wind vector in box
    fig.add_annotation(
        x=true_wind_arrow_origin[0],
        y=true_wind_arrow_origin[1],
        ax=true_wind_arrow_origin[0] + tw_dx,
        ay=true_wind_arrow_origin[1] + tw_dy,
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
        hovertext=(f"<b>🌬️ True Wind</b><br>" f"speed: {tw_mag:.2f} kmph<br>"),
        hoverlabel=dict(bgcolor="white"),
    )

    # boat vector in box
    fig.add_annotation(
        x=boat_arrow_origin[0],
        y=boat_arrow_origin[1],
        ax=boat_arrow_origin[0] + bw_dx,
        ay=boat_arrow_origin[1] + bw_dy,
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
        hovertext=(f"<b>🛶 Boat Wind</b><br>" f"speed: {bw_mag:.2f} kmph<br>"),
        hoverlabel=dict(bgcolor="white"),
    )

    # Fill in box
    fig.add_shape(
        type="rect",
        xref="paper",
        yref="paper",
        x0=0.76,
        y0=0.00,
        x1=0.99,
        y1=0.22,
        fillcolor="white",
        line=dict(width=4),
    )

    # add boat and true wind labels
    fig.add_annotation(
        x=-8,
        y=4,
        xref="x2",
        yref="y2",
        text=f"Boat - {bw_mag:.2f} kmph",
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
        text=f"True - {tw_mag:.2f} kmph",
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
        text=f"Apparent - {aw_mag:.2f} kmph",
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
        x=6,
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
        x0=x_min,
        y0=y_min,
        x1=x_max,
        y1=y_max,
        fillcolor="rgba(000, 100, 255, 0.25)",  # light red, semi-transparent
        line=dict(width=0),
        layer="below",
    )

    # Path trace (path to goal point)
    if state.final_local_wp_x and state.final_local_wp_y:
        planned_x = list(state.final_local_wp_x)
        planned_y = list(state.final_local_wp_y)
        path_trace = go.Scatter(
            x=planned_x,
            y=planned_y,
            mode="lines",
            name="Path to Goal",
            line=dict(width=2, dash="dot", color="blue"),
            hovertemplate="X: %{x:.2f}<br>Y: %{y:.2f}<extra></extra>",
        )

    # Add all traces to the figure
    fig.add_trace(intermediate_trace)
    fig.add_trace(goal_trace)
    fig.add_trace(boat_trace)
    fig.add_trace(path_trace)

    # Display AIS Ships
    for x_val, y_val, heading, ais_id, speed in zip(
        state.ais_pos_x, state.ais_pos_y, state.ais_headings, state.ais_ship_ids, state.ais_speeds
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
                    f"Speed: {speed:.1f} km/h<br>"
                ),
                marker=dict(
                    symbol="arrow",
                    color="orange",
                    line=dict(width=2, color="DarkSlateGrey"),
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
                    showlegend=False,
                    opacity=0.5,
                )
            )

    # Display message if local goal point has changed
    if msg_to_display:
        fig.add_annotation(
            text=msg_to_display,
            xref="paper",
            yref="paper",
            x=0.02,
            y=0.98,
            showarrow=False,
            bgcolor="rgba(255,230,150,0.9)",
            bordercolor="rgba(0,0,0,0.2)",
            borderwidth=1,
        )

    # Update Layout
    x_min, y_min, x_max, y_max = state_space.bounds
    fig.update_layout(
        title="Path Planning",
        xaxis_title="X Coordinate",
        yaxis_title="Y Coordinate",
        xaxis=dict(
            range=[x_min, x_max],
            domain=[0.0, 0.98],
        ),
        yaxis=dict(
            range=[y_min, y_max],
            domain=[0.25, 1.0],
        ),
        legend=dict(x=0, y=1),  # Position the legend at the top left
        showlegend=True,
        uirevision="constant",
    )

    return fig
