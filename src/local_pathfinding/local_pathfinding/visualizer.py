"""
Sailbot Path Planning Visualizer

This script provides visualization tools for the sailbot path planning module. This script sets up
a Dash web application to visualize sailbot's path planning data in real time. It processes ROS
messages such as waypoint coordinates and GPS data, converting them from latitude and longitude to
Cartesian coordinates in kilometers.

Main Components:
1. VisualizerState: A class that processes the ROS messages and converts coordinates.
2. Dash App: A web application that displays the path planning data in real-time.
3. Plotting Functions: Functions to create and update the plots based on the processed data.
4. Callbacks: Functions that update the plots at regular intervals.
"""

import math
from collections import deque
from dataclasses import dataclass
from multiprocessing import Queue
from typing import Any, Dict, List, Optional, Tuple

import custom_interfaces.msg as ci
import dash
import plotly.graph_objects as go
from dash import dcc, html
from dash.dependencies import Input, Output, State
from shapely.geometry import MultiPolygon, Polygon

import local_pathfinding.coord_systems as cs
import local_pathfinding.wind_coord_systems as wcs
from local_pathfinding.ompl_path import OMPLPath

UPDATE_INTERVAL_MS = 2500
DEFAULT_PLOT_RANGE = [-100, 100]
BOX_BUFFER_SIZE_KM = 1.0

WIND_BOX_X_DOMAIN = (0.76, 0.99)
WIND_BOX_Y_DOMAIN = (0.00, 0.22)
WIND_BOX_RANGE = (-10, 10)

WIND_ARROW_LEN = 4.0
WIND_BOX_ORIGIN_XY = cs.XY(6.0, 0.0)
WIND_BOX_Y_OFFSET = 0.5

GOAL_CHANGE_ROUND_DECIMALS = 3  # avoid float jitter spam

app = dash.Dash(__name__)
queue: Optional[Queue] = None  # type: ignore


@dataclass(frozen=True)
class GoalChange:
    """
    Stores pop-up message if the local goal waypoint has changed between updates.

    Attributes:
        new_goal_xy_rounded: The current goal position (x, y) rounded to a fixed precision
            to prevent message spam from float jitter.
        message: A human-readable message to show on the plot, or None if no change occurred.
    """

    new_goal_xy_rounded: Tuple[float, float]
    message: Optional[str]


@dataclass(frozen=True)
class AISShipData:
    """
    Stores data for a single AIS ship.

    Attributes:
        pos_x_km: X coordinate (km) of the AIS ship.
        pos_y_km: Y coordinate (km) of the AIS ship.
        heading_deg: Heading (degrees) of the AIS ship.
        speed_kmph: Speed (km/h) of the AIS ship.
    """

    pos_x_km: float
    pos_y_km: float
    heading_deg: float
    speed_kmph: float


@dataclass(frozen=True)
class WindBoxConfig:
    """
    Stores configuration data for the wind box inset in the plot.

    Attributes:
        layout_config: Dict[str, Any]: Configuration for the wind box axes in fig.update_layout().
        wind_arrows: List[Dict[str, Any]]: List of arrow annotation dicts for wind vectors in
                                        fig.add_annotation().
        background_info: Dict[str, Any]: Dict for the background rectangle shape in
                                    fig.add_shape().
        annotations: List[Dict[str, Any]]: List of text annotation dicts for wind vector labels in
                                        fig.add_annotation().
    """

    layout_config: Dict[str, Any]
    wind_arrows: List[Dict[str, Any]]
    background_info: Dict[str, Any]
    annotations: List[Dict[str, Any]]


class VisualizerState:
    """
    Converts the ROS message to a format that can be used by the visualizer.
    This class extracts the latest message. It converts latitude/longitude to XY Cartesian
    coordinates (km) using a goal-anchored reference frame, and prepares paths, obstacles,
    AIS ships, and wind vectors in forms that can be directly used by Plotly.

    Coordinate conventions:
        - XY coordinates are in kilometers.
        - Headings are degrees in the navigation convention (0° = North, +90° = East)

    Attributes:
        sailbot_pos_x_km (List[float]): X coordinates (km) of the boat track.
        sailbot_pos_y_km (List[float]): Y coordinates (km) of the boat track.

        final_local_wp_x_km (List[float]): X coordinates (km) of the latest local path waypoints.
        final_local_wp_y_km (List[float]): Y coordinates (km) of the latest local path waypoints.

        sailbot_gps (List[ci.Gps]): GPS messages used for heading (degrees) and speed (km/h)
                                    display.

        ais_ships_by_id (Dict[int, AISShipData]): Dictionary mapping AIS ship IDs to their data
                                                   (position, heading, speed).

        land_obstacles_xy (List[Polygon]): Land obstacle polygons in XY (km).
        boat_obstacles_xy (List[Polygon]): Boat collision-zone polygons in XY (km).

        aw_vector_kmph (cs.XY): Apparent wind vector in global XY.
        tw_vector_kmph (cs.XY): True wind vector in global XY.
        bw_vector_kmph (cs.XY): Boat velocity vector in global XY.
    """

    def __init__(self, msgs: deque[ci.LPathData]):
        if not msgs:
            raise ValueError("VisualizerState requires at least one message")

        self.latest_msg = msgs[-1]
        self._validate_message(self.latest_msg)

        # Boat history
        self.sailbot_lat_lon = [msg.gps.lat_lon for msg in msgs]
        self.sailbot_gps = [msg.gps for msg in msgs]

        # Paths
        self.all_local_wp = [msg.local_path.waypoints for msg in msgs]
        self.global_path = self.latest_msg.global_path

        self.reference_lat_lon = self.global_path.waypoints[-1]
        self.sailbot_xy_km = cs.latlon_list_to_xy_list(
            self.reference_lat_lon, self.sailbot_lat_lon
        )
        self.all_wp_xy = [
            cs.latlon_list_to_xy_list(self.reference_lat_lon, waypoints)
            for waypoints in self.all_local_wp
        ]
        self.sailbot_pos_x_km, self.sailbot_pos_y_km = self._split_coordinates(self.sailbot_xy_km)
        self.final_local_wp_x_km, self.final_local_wp_y_km = self._split_coordinates(
            self.all_wp_xy[-1]
        )
        self.all_local_wp_x, self.all_local_wp_y = zip(
            *[self._split_coordinates(waypoints) for waypoints in self.all_wp_xy]
        )

        # AIS ships
        self.ais_ships = self.latest_msg.ais_ships.ships
        ais_ship_latlons = [ship.lat_lon for ship in self.ais_ships]
        ais_ship_xy = cs.latlon_list_to_xy_list(self.reference_lat_lon, ais_ship_latlons)
        ais_positions = self._split_coordinates(ais_ship_xy)

        self.ais_ships_by_id: Dict[int, AISShipData] = {}
        for ship, (x, y) in zip(self.ais_ships, zip(ais_positions[0], ais_positions[1])):
            self.ais_ships_by_id[ship.id] = AISShipData(
                pos_x_km=x, pos_y_km=y, heading_deg=ship.cog.heading, speed_kmph=ship.sog.speed
            )

        # Obstacles
        # Process land obstacles
        self.land_obstacles_xy = self._process_obstacles_by_type(
            self.latest_msg.obstacles, self.reference_lat_lon, "Land"
        )
        # Process Boat Obstacles
        self.boat_obstacles_xy = self._process_obstacles_by_type(
            self.latest_msg.obstacles, self.reference_lat_lon, "Boat"
        )

        # Wind Vectors
        boat_speed_kmph = self.latest_msg.gps.speed.speed
        boat_heading_deg = self.latest_msg.gps.heading.heading
        aw_speed_kmph = self.latest_msg.filtered_wind_sensor.speed.speed
        aw_dir_boat_deg = self.latest_msg.filtered_wind_sensor.direction

        # Convert Apparent wind to global frame
        aw_dir_global_deg = wcs.boat_to_global_coordinate(boat_heading_deg, aw_dir_boat_deg)
        aw_dir_global_rad = math.radians(aw_dir_global_deg)
        # Compute apparent wind vector (in global frame)
        self.aw_vector_kmph = cs.polar_to_cartesian(aw_dir_global_rad, aw_speed_kmph)

        # True wind from apparent
        tw_angle_rad, tw_speed_kmph = wcs.get_true_wind(
            aw_dir_global_deg, aw_speed_kmph, boat_heading_deg, boat_speed_kmph
        )
        self.tw_vector_kmph = cs.polar_to_cartesian(tw_angle_rad, tw_speed_kmph)

        # Boat wind vector
        boat_wind_radians = math.radians(cs.bound_to_180(boat_heading_deg + 180))
        self.bw_vector_kmph = cs.polar_to_cartesian(boat_wind_radians, boat_speed_kmph)

    @staticmethod
    def _validate_message(msg: ci.LPathData) -> None:
        """
        Checks if the sailbot observer node received any messages.
        If not, it raises a ValueError.
        Args:
            msg: Latest `ci.LPathData` message to validate.
        """
        if msg.global_path is None:
            raise ValueError("No global path received in the message")
        if msg.local_path is None:
            raise ValueError("No local path received in the message")
        if msg.gps is None:
            raise ValueError("No GPS data received in the message")

    @staticmethod
    def _split_coordinates(positions) -> Tuple[List[float], List[float]]:
        """
        Splits a list of XY objects into their separate x and y components.
        Args:
            positions: Iterable of objects with `.x` and `.y` attributes (e.g., `cs.XY`).

        Returns:
            (x_coords, y_coords): Two lists of floats suitable for Plotly traces.
        """
        x_coords = [pos.x for pos in positions]
        y_coords = [pos.y for pos in positions]
        return x_coords, y_coords

    @staticmethod
    def _process_obstacles_by_type(obstacles, reference, obstacle_type: str) -> List[Polygon]:
        """
        Convert obstacles of a specific type (specifically Boat and Land here) into Shapely
        polygons in the XY frame.

        Args:
            obstacles: Iterable of obstacle messages (or None).
            reference: Lat/Lon reference used for conversion to XY.
            obstacle_type: Exact obstacle type string to include (e.g. "Land", "Boat").

        Returns:
            List of Shapely Polygons for obstacles of the given type.
        """
        processed: List[Polygon] = []
        if obstacles is None:
            return processed

        for obstacle in obstacles:
            if obstacle.obstacle_type != obstacle_type:
                continue

            xy_points = []
            for pt in obstacle.points:
                xy = cs.latlon_to_xy(reference, pt)
                xy_points.append((xy.x, xy.y))

            if len(xy_points) >= 3:
                processed.append(Polygon(xy_points))

        return processed


# --------------------------------------
# Math Helpers
# --------------------------------------
def get_unit_vector(vec: cs.XY) -> cs.XY:
    """
    Normalize a 2D vector to unit length.

    Args:
        vec: Vector in XY form.

    Returns:
        A unit vector with the same direction as `vec`. If magnitude is really small (~0), it
        returns (0.0, 0.0).
    """
    mag = math.hypot(vec.x, vec.y)
    if mag > 1e-6:
        return cs.XY(vec.x / mag, vec.y / mag)
    return cs.XY(0.0, 0.0)


def compute_goal_change(
    last_goal_xy_km: Optional[Tuple[float, float]], goal_xy_km: Tuple[float, float]
) -> GoalChange:
    """
    Determine whether the local goal moved since the last update (with some jitter tolerance).

    Args:
        last_goal_xy_km: Previously stored (x, y) goal in km (already rounded), or None if rendered
                      for the first time.
        goal_xy_km: Current (x, y) goal in km.

    Returns:
        GoalChange containing the rounded goal coordinates and an optional popup message.
    """
    rounded = (
        round(goal_xy_km[0], GOAL_CHANGE_ROUND_DECIMALS),
        round(goal_xy_km[1], GOAL_CHANGE_ROUND_DECIMALS),
    )
    msg = None
    if last_goal_xy_km is not None and rounded != last_goal_xy_km:
        msg = f"Local goal advanced to ({rounded[0]}, {rounded[1]})"
    return GoalChange(new_goal_xy_rounded=rounded, message=msg)


# --------------------------------------
# Figure Builders
# --------------------------------------
def initial_plot() -> go.Figure:
    """
    Create a base Plotly figure with default axis titles and ranges.

    Returns:
        A Plotly figure ready for adding traces and layout updates.
    """
    fig = go.Figure()
    fig = go.FigureWidget(fig)

    fig.update_layout(
        xaxis_title="X (Km)",
        yaxis_title="Y (Km)",
        xaxis=dict(range=DEFAULT_PLOT_RANGE),
        yaxis=dict(range=DEFAULT_PLOT_RANGE),
    )
    return fig


def build_intermediate_trace(local_x_km: List[float], local_y_km: List[float]) -> go.Scatter:
    """
    Create the scatter trace for intermediate local waypoints (excluding start and goal).

    Args:
        local_x_km: X coordinates of the local waypoint list (km).
        local_y_km: Y coordinates of the local waypoint list (km).

    Returns:
        A Plotly Scatter trace containing marker with text labels for intermediate waypoints.
        If fewer than 3 points exist, returns an empty trace (i.e., no intermediate points).
    """
    if len(local_x_km) < 3:
        return go.Scatter(x=[], y=[], mode="markers+text", name="Intermediate")
    labels = [f"LW{i+1}" for i, _ in enumerate(local_x_km[1:-1])]
    return go.Scatter(
        x=local_x_km[1:-1],
        y=local_y_km[1:-1],
        mode="markers+text",
        marker=dict(color="blue", size=8),
        text=labels,
        textposition="top center",
        name="Intermediate",
    )


def build_goal_trace(goal_xy_km: Tuple[float, float], angle_deg: float) -> go.Scatter:
    """
    Create the marker trace for the local goal waypoint.

    Args:
        goal_xy_km: (x, y) goal position in km.
        angle_deg: Angle from boat to goal in degrees (visualizer convention) used in hover text.

    Returns:
        A Plotly Scatter trace representing the goal point.
    """
    return go.Scatter(
        x=[goal_xy_km[0]],
        y=[goal_xy_km[1]],
        mode="markers",
        marker=dict(color="red", size=10),
        name="Goal",
        hovertemplate="X: %{x:.2f} <br>"
        + "Y: %{y:.2f} <br>"
        + "Angle from the boat: "
        + f"{angle_deg:.1f}°"
        + "<extra></extra>",
    )


def build_path_trace(
    local_x_km: List[float], local_y_km: List[float], boat_xy_km: Tuple[float, float]
) -> Optional[go.Scatter]:
    """
    Create a dotted line trace connecting the local waypoints to the goal.

    Args:
        local_x_km: Local path X coordinates in km.
        local_y_km: Local path Y coordinates in km.
        boat_xy_km: Sailboat's latest (X, Y) coordinates in km.

    Returns:
        A Plotly Scatter trace for the path line, or None if input is empty.
    """
    if not local_x_km or not local_y_km:
        return None
    return go.Scatter(
        x=list(local_x_km),
        y=list(local_y_km),
        mode="lines",
        name="Path to Goal",
        line=dict(width=2, dash="dot", color="blue"),
        hovertemplate="X: %{x:.2f}<br>Y: %{y:.2f}<extra></extra>",
    )


def build_boat_trace(
    state: VisualizerState, boat_xy_km: Tuple[float, float], dist_to_goal_km: float
) -> go.Scatter:
    """
    Create the boat marker trace (filled arrow-head/ triangle) at the current boat position.

    Args:
        state: VisualizerState containing GPS heading/speed history.
        boat_xy_km: (x, y) current boat position in km.
        dist_to_goal_km: Current straight-line distance to the goal in km (for hover text).

    Returns:
        A Plotly Scatter trace representing the boat marker with heading-based rotation.
    """
    heading = state.sailbot_gps[-1].heading.heading
    speed = state.sailbot_gps[-1].speed.speed
    return go.Scatter(
        x=[boat_xy_km[0]],
        y=[boat_xy_km[1]],
        mode="markers",
        name="Boat",
        hovertemplate=(
            "<b>🚢 Sailbot Current Position</b><br>"
            "X: %{x:.2f} <br>"
            "Y: %{y:.2f} <br>"
            "Heading: " + f"{heading:.1f}°<br>"
            f"Speed: {speed:.1f} km/h <br>"
            f"Distance to Goal: {dist_to_goal_km:.2f} km<br>"
            "<extra></extra>"
        ),
        marker=dict(
            symbol="arrow",
            color="yellow",
            line=dict(width=2, color="DarkSlateGrey"),
            size=20,
            angleref="up",
            angle=cs.true_bearing_to_plotly_cartesian(heading),
        ),
    )


def build_polygon_traces(
    polys: List[Polygon],
    *,
    name: str,
    line: dict,
    fillcolor: str,
    opacity: float = 0.5,
    hoverinfo: Optional[str] = None,
    showlegend: bool = True,
) -> List[go.Scatter]:
    """
    Build filled polygon trace overlays for the figure.
    This is used for land obstacles and boat collision zones.

    Args:
        polys: List of Shapely polygons in XY (km).
        name: Legend name for the polygons.
        line: Plotly line dict for polygon edges (e.g., {"color": "lightgreen"}).
        fillcolor: Plotly fill color for the polygon interior.
        opacity: Opacity for the fill/trace.
        hoverinfo: Optional Plotly hoverinfo mode (e.g., "skip") to disable hover.
        showlegend: Whether this trace should appear in the legend.

    Returns:
        List of Plotly Scatter traces for the polygons.
    """
    traces = []
    for poly in polys:
        if poly.is_empty:
            continue

        x = list(poly.exterior.xy[0])
        y = list(poly.exterior.xy[1])

        scatter_kwargs = dict(
            x=x,
            y=y,
            fill="toself",
            mode="lines",
            line=line,
            fillcolor=fillcolor,
            opacity=opacity,
            name=name,
            showlegend=showlegend,
        )
        if hoverinfo is not None:
            scatter_kwargs["hoverinfo"] = hoverinfo

        traces.append(go.Scatter(**scatter_kwargs))

    return traces


def build_ais_traces(state: VisualizerState) -> List[go.Scatter]:
    """
    Build AIS ship markers (filled arrow-heads/ triangles) for the plot.

    Args:
        state: VisualizerState containing AIS ship data by ID.

    Returns:
        List of Plotly Scatter traces for AIS ships.
    """
    traces = []
    for ais_id, ship_data in state.ais_ships_by_id.items():
        x_val = ship_data.pos_x_km
        y_val = ship_data.pos_y_km
        heading = ship_data.heading_deg
        speed = ship_data.speed_kmph

        traces.append(
            go.Scatter(
                x=[x_val],
                y=[y_val],
                mode="markers",
                name=f"AIS {ais_id}",
                hovertemplate=(
                    f"<b>🚢 AIS Ship {ais_id}</b><br>"
                    f"X: {x_val:.2f}<br>"
                    f"Y: {y_val:.2f}<br>"
                    f"Heading: {heading:.1f}°<br>"
                    f"Speed: {speed:.1f} km/h<br>"
                    "<extra></extra>"
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
    return traces


def configure_wind_box_elements(state: VisualizerState) -> WindBoxConfig:
    """
    Build the "wind box" inset configuration with scaled wind and boat velocity vectors.

    Constructs all components needed to render the wind box inset, including:
    - Secondary axes configuration (x2/y2) for the inset domain and range.
    - Normalized/scaled arrow annotations for apparent wind, true wind, and boat velocity.
    - Background rectangle shape for the inset.
    - Text annotations for wind vector labels and speed values.

    Args:
        state: VisualizerState containing wind vectors in global XY frame.

    Returns:
        WindBoxConfig containing:
            - layout_config: Dict of axis configuration for fig.update_layout()
            - wind_arrows: List of arrow annotation dicts for fig.add_annotation()
            - background_info: Dict for background rect shape to add via fig.add_shape()
            - annotations: List of text annotation dicts for fig.add_annotation()
    """
    # configure wind box axis
    layout_config = {
        "xaxis2": dict(
            domain=list(WIND_BOX_X_DOMAIN),
            anchor="y2",
            range=list(WIND_BOX_RANGE),
            showgrid=False,
            zeroline=True,
            visible=False,
            fixedrange=True,
        ),
        "yaxis2": dict(
            domain=list(WIND_BOX_Y_DOMAIN),
            anchor="x2",
            range=list(WIND_BOX_RANGE),
            showgrid=False,
            zeroline=True,
            visible=False,
            fixedrange=True,
        ),
    }

    # Re-Calculating vectors for better scaling in the wind box inset.
    aw_unit = get_unit_vector(state.aw_vector_kmph)
    tw_unit = get_unit_vector(state.tw_vector_kmph)
    bw_unit = get_unit_vector(state.bw_vector_kmph)

    aw_mag = math.hypot(state.aw_vector_kmph.x, state.aw_vector_kmph.y)
    tw_mag = math.hypot(state.tw_vector_kmph.x, state.tw_vector_kmph.y)
    bw_mag = math.hypot(state.bw_vector_kmph.x, state.bw_vector_kmph.y)

    aw_dx, aw_dy = aw_unit.x * WIND_ARROW_LEN, aw_unit.y * WIND_ARROW_LEN
    tw_dx, tw_dy = tw_unit.x * WIND_ARROW_LEN, tw_unit.y * WIND_ARROW_LEN
    bw_dx, bw_dy = bw_unit.x * WIND_ARROW_LEN, bw_unit.y * WIND_ARROW_LEN

    origin_x, origin_y = WIND_BOX_ORIGIN_XY
    boat_origin = (origin_x, origin_y + WIND_BOX_Y_OFFSET)
    true_origin = (origin_x, origin_y)
    app_origin = (origin_x, origin_y - WIND_BOX_Y_OFFSET)

    wind_arrows = []

    # Function for adding re-calculated wind vectors arrows to the wind box inset
    def add_arrow(origin, dx, dy, color, title, mag):
        wind_arrows.append(
            {
                "x": origin[0],
                "y": origin[1],
                "ax": origin[0] - dx,
                "ay": origin[1] - dy,
                "xref": "x2",
                "yref": "y2",
                "axref": "x2",
                "ayref": "y2",
                "showarrow": True,
                "arrowhead": 3,
                "arrowsize": 0.5,
                "arrowwidth": 3,
                "arrowcolor": color,
                "standoff": 2,
                "text": "",
                "hovertext": (f"<b>{title}</b><br>" f"speed: {mag:.2f} kmph<br>"),
                "hoverlabel": dict(bgcolor="white"),
            }
        )

    add_arrow(app_origin, aw_dx, aw_dy, "purple", "️Apparent Wind", aw_mag)
    add_arrow(true_origin, tw_dx, tw_dy, "blue", "️True Wind", tw_mag)
    add_arrow(boat_origin, bw_dx, bw_dy, "red", "Boat Wind", bw_mag)

    # Wind box background
    background_shape = {
        "type": "rect",
        "xref": "paper",
        "yref": "paper",
        "x0": WIND_BOX_X_DOMAIN[0],
        "y0": WIND_BOX_Y_DOMAIN[0],
        "x1": WIND_BOX_X_DOMAIN[1],
        "y1": WIND_BOX_Y_DOMAIN[1],
        "fillcolor": "white",
        "line": {"width": 4},
    }

    # labels for wind vectors and boat vector
    annotations = [
        {
            "x": -8,
            "y": 4,
            "xref": "x2",
            "yref": "y2",
            "text": f"Boat - {bw_mag:.2f} kmph",
            "showarrow": False,
            "align": "left",
            "xanchor": "left",
            "font": {"size": 12, "color": "red"},
        },
        {
            "x": -8,
            "y": 0,
            "xref": "x2",
            "yref": "y2",
            "text": f"True - {tw_mag:.2f} kmph",
            "showarrow": False,
            "align": "left",
            "xanchor": "left",
            "font": {"size": 12, "color": "blue"},
        },
        {
            "x": -8,
            "y": -4,
            "xref": "x2",
            "yref": "y2",
            "text": f"Apparent - {aw_mag:.2f} kmph",
            "showarrow": False,
            "align": "left",
            "xanchor": "left",
            "font": {"size": 12, "color": "purple"},
        },
        {
            "x": 0,
            "y": 6,
            "xref": "x2",
            "yref": "y2",
            "showarrow": False,
            "text": "<b>Wind</b>",
            "font": {"size": 12, "color": "black"},
        },
        {
            "x": 6,
            "y": 0,
            "xref": "x2",
            "yref": "y2",
            "showarrow": False,
            "text": "●",
            "font": {"size": 12, "color": "lightgreen"},
        },
    ]
    return WindBoxConfig(layout_config, wind_arrows, background_shape, annotations)


def compute_and_add_state_space(
    boat_xy_km: Tuple[float, float],
    goal_xy_km: Tuple[float, float],
    fig: go.Figure,
    zoom_needed: bool,
    last_range: Optional[Dict[str, List[float]]]
):
    """
    Build the visualization state-space overlay around the boat and goal. Then, add the built
    rectangular overlay spanning the bounds of the state space and returns the state-space.

    The overlay is a MultiPolygon consisting of:
    - A buffer box around the boat
    - A buffer box around the goal

    Args:
        boat_xy_km: (x, y) boat position in km.
        goal_xy_km: (x, y) goal position in km.
        fig: Target Plotly figure.
    """
    boat_pos = cs.XY(boat_xy_km[0], boat_xy_km[1])
    goal_pos = cs.XY(goal_xy_km[0], goal_xy_km[1])

    boat_box = OMPLPath.create_buffer_around_position(boat_pos, BOX_BUFFER_SIZE_KM)
    goal_box = OMPLPath.create_buffer_around_position(goal_pos, BOX_BUFFER_SIZE_KM)
    state_space = MultiPolygon([boat_box, goal_box])

    # Adding the calculated rectangular overlay(state_space) to the plot
    x_min, y_min, x_max, y_max = state_space.bounds
    fig.add_shape(
        type="rect",
        x0=x_min,
        y0=y_min,
        x1=x_max,
        y1=y_max,
        fillcolor="rgba(000, 100, 255, 0.25)",
        line=dict(width=0),
        layer="below",
    )

    if not zoom_needed and last_range is not None:
        fig.update_layout(
            xaxis=dict(range=last_range["x"],
                       autorange=False),
            yaxis=dict(range=last_range["y"],
                       autorange=False),
        )
    else:
        fig.update_layout(
            xaxis=dict(range=[x_min, x_max],
                       autorange=False),
            yaxis=dict(range=[y_min, y_max],
                       autorange=False),
        )


def add_goal_change_popup(fig: go.Figure, message: Optional[str]) -> None:
    """
    Adds a popup annotation to the top-left of the window when the local goal advances.

    Args:
        fig: Target Plotly figure.
        message: Popup text to display; if None/empty, nothing is added.
    """
    if not message:
        return
    fig.add_annotation(
        text=message,
        xref="paper",
        yref="paper",
        x=0.02,
        y=0.98,
        showarrow=False,
        bgcolor="rgba(255,230,150,0.9)",
        bordercolor="rgba(0,0,0,0.2)",
        borderwidth=1,
    )


def apply_layout(fig: go.Figure) -> None:
    """
    Apply the main plot layout configuration (axis titles, domains, legend, and optional ranges).

    Args:
        fig: Target Plotly figure.
    """
    xaxis = dict(domain=[0.0, 0.98])
    yaxis = dict(domain=[0.30, 1.0])

    fig.update_layout(
        xaxis_title="X (Km)",
        yaxis_title="Y (Km)",
        font=dict(color="rgb(18, 70, 139)"),
        xaxis=xaxis,
        yaxis=yaxis,
        legend=dict(orientation="h", y=1.15, x=0.5, xanchor="center"),
        showlegend=True,
        uirevision="constant",
    )


def build_figure(
    state: VisualizerState,
    last_goal_xy_km: Optional[Tuple[float, float]],
    last_range: Optional[Dict[str, List[float]]],
) -> Tuple[go.Figure, Tuple[float, float]]:
    """
    Builds and renders the complete path planning visualization figure.

    This function orchestrates all visual elements: boat state, local path, obstacles,
    AIS ships, wind vectors, and state-space overlay.

    Args:
        state: VisualizerState containing processed ROS message data (boat position, path,
               obstacles, AIS ships, wind vectors).
        last_goal_xy_km: Previous goal position (x, y) in km, or None on first render. Used to
                      detect goal changes and show a popup message.

    Returns:
        (fig, new_goal_xy_rounded):
            - fig: Updated Plotly figure ready for display.
            - new_goal_xy_rounded: Current goal position rounded to GOAL_CHANGE_ROUND_DECIMALS
                                for float jitter tolerance.

    Raises:
        ValueError: If no local waypoints are available for plotting.
    """
    fig = initial_plot()

    local_x_km = list(state.final_local_wp_x_km)
    local_y_km = list(state.final_local_wp_y_km)

    # Boat and goal info
    boat_xy_km = cs.XY(state.sailbot_pos_x_km[-1], state.sailbot_pos_y_km[-1])

    if not local_x_km or not local_y_km:
        raise ValueError("No local waypoints available for plotting")
    goal_xy_km = cs.XY(local_x_km[-1], local_y_km[-1])
    goal_change = compute_goal_change(last_goal_xy_km, goal_xy_km)

    # Computing angle and distance from boat to goal
    angle_deg = math.degrees(
        math.atan2(goal_xy_km[0] - boat_xy_km[0], goal_xy_km[1] - boat_xy_km[1])
    )
    dist_km = math.hypot(goal_xy_km[0] - boat_xy_km[0], goal_xy_km[1] - boat_xy_km[1])

    # adding all the Traces(intermediate, goal, boat and path) to the plot
    fig.add_trace(build_intermediate_trace(local_x_km, local_y_km))
    fig.add_trace(build_goal_trace(goal_xy_km, angle_deg))
    fig.add_trace(build_boat_trace(state, boat_xy_km, dist_km))
    path_trace = build_path_trace(local_x_km, local_y_km, boat_xy_km)
    if path_trace is not None:
        fig.add_trace(path_trace)

    # Adding Obstacle (both Land and Boat) and AIS ships to the plot
    land_traces = build_polygon_traces(
        state.land_obstacles_xy,
        name="Land Obstacle",
        line={"color": "lightgreen"},
        fillcolor="lightgreen",
        opacity=0.5,
        showlegend=True,
    )
    boat_traces = build_polygon_traces(
        state.boat_obstacles_xy,
        name="AIS Collision Zone",
        line={"width": 2},
        fillcolor="rgba(255,165,0,0.25)",
        opacity=0.5,
        hoverinfo="skip",
        showlegend=False,
    )
    ais_traces = build_ais_traces(state)
    fig.add_traces(land_traces + boat_traces + ais_traces)

    # Creating Wind box and its elements and adding them to the plot
    wind_config = configure_wind_box_elements(state)
    fig.update_layout(**wind_config.layout_config)
    for arrow in wind_config.wind_arrows:
        fig.add_annotation(arrow)
    fig.add_shape(**wind_config.background_info)
    for annotation in wind_config.annotations:
        fig.add_annotation(annotation)

    # Computing State space overlay and adding it to the plot
    zoom_needed = last_goal_xy_km is None
    compute_and_add_state_space(boat_xy_km, goal_xy_km, fig, zoom_needed, last_range)
    apply_layout(fig)
    add_goal_change_popup(fig, goal_change.message)  # Popup message for goal change
    return fig, goal_change.new_goal_xy_rounded


# -----------------------------
# Dash App entry points
# -----------------------------
def dash_app(q: Queue):
    """
    Creates and launches the Dash application and attach the shared ROS message queue.

    Args:
        q: Multiprocessing Queue which provides `VisualizerState` objects from the ROS node.
    """
    # Allows it to be accessed in the callbacks
    global queue
    queue = q

    app.layout = html.Div(
        style={"height": "100vh", "width": "100vw", "margin": 0, "padding": 0},
        children=[
            html.H2(
                "UBC Sailbot Pathfinding",
                style={"fontFamily": "Consolas, monospace", "color": "rgb(18, 70, 139)"},
            ),
            dcc.Graph(id="live-graph", style={"height": "90vh", "width": "100%"}),
            dcc.Interval(id="interval-component", interval=UPDATE_INTERVAL_MS, n_intervals=0),
            dcc.Store(id="goal-store", data=None),
            dcc.Store(id="range-store", data=None),
            html.Button(
                "Reset the view to state space",
                id="reset-button",
                n_clicks=0,
                style={
                    "position": "absolute",
                    "top": "20px",
                    "right": "20px",
                    "padding": "10px 20px",
                    "zIndex": 1000,
                },
            ),
        ],
    )

    app.run(debug=True, use_reloader=False)


@app.callback(
    Output("live-graph", "figure"),
    Output("goal-store", "data"),
    Output("range-store", "data"),
    Input("interval-component", "n_intervals"),
    Input("live-graph", "relayoutData"),
    Input("reset-button", "n_clicks"),
    State("live-graph", "figure"),
    State("goal-store", "data"),
    State("range-store", "data"),
    prevent_initial_call=True,
)
def update_graph(_: int,
                 relayoutData,
                 __: int,
                 current_figure,
                 last_goal_xy_km: Optional[List[float]],
                 stored_range):
    """
    Dash callback: handles both interval updates and reset button clicks.
    Uses callback_context to determine which input triggered the update.

    Args:
        _: Dash interval tick (unused).
        relayoutData: Data relayed from Plotly when user pans, zooms in/out, autoscales
        __: Reset button n_clicks (unused).
        current_figure: Current figure state from live-graph.
        last_goal_xy_km: Previously stored goal as [x, y] or None on first run.
        stored_range: Previously stored range as {"x": [xmin, xmax], "y": [ymin, ymax]}
                      or None on first run

    Returns:
        (fig, new_goal_as_list, last_range):
            - fig: The updated Plotly figure
            - new_goal_as_list: [x, y] for storage in dcc.Store (JSON serializable)
            - last_range: [x-range, y-range] for storage in dcc.Store (JSON serializable)

    """
    global queue

    if relayoutData and "xaxis.range[0]" in relayoutData:
        last_range = {
            "x": [
                relayoutData["xaxis.range[0]"],
                relayoutData["xaxis.range[1]"],
            ],
            "y": [
                relayoutData["yaxis.range[0]"],
                relayoutData["yaxis.range[1]"],
            ],
        }
    else:
        last_range = stored_range

    ctx = dash.callback_context

    triggered_id = ctx.triggered[0]["prop_id"].split(".")[0] if ctx.triggered else None

    if triggered_id == "reset-button":
        if current_figure is None:
            return dash.no_update, dash.no_update, dash.no_update

        fig = go.Figure(current_figure)
        fig.update_layout(
            xaxis=dict(range=DEFAULT_PLOT_RANGE, autorange=False),
            yaxis=dict(range=DEFAULT_PLOT_RANGE, autorange=False),
            uirevision="reset",
        )
        return fig, last_goal_xy_km, last_range

    # Interval update (default behavior)
    if queue is None or queue.empty():
        return dash.no_update, dash.no_update, dash.no_update

    state = queue.get()  # type: ignore
    last_goal_tuple = (
        cs.XY(last_goal_xy_km[0], last_goal_xy_km[1]) if last_goal_xy_km is not None else None
    )

    fig, new_goal_xy = build_figure(state, last_goal_tuple, last_range)
    return fig, [new_goal_xy[0], new_goal_xy[1]], last_range
