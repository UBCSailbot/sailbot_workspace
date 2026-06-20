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
import subprocess
from collections import deque
from dataclasses import dataclass
from io import BytesIO
from multiprocessing import Queue
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import dash
import plotly.graph_objects as go
import requests  # type: ignore[import-untyped]
import yaml
from dash import dcc, html
from dash.dependencies import Input, Output, State
from PIL import Image
from shapely.geometry import MultiPolygon, Polygon

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs
import local_pathfinding.wind_coord_systems as wcs
from local_pathfinding.ompl_path import OMPLPath
from local_pathfinding.ompl_validity import NO_GO_ZONE

UPDATE_INTERVAL_MS = 2500
DEFAULT_PLOT_RANGE = [-100.0, 100.0]
BOX_BUFFER_SIZE_KM = 1.0

WIND_BOX_X_DOMAIN = (0.76, 0.99)
WIND_BOX_Y_DOMAIN = (0.00, 0.22)
WIND_BOX_RANGE = (-10, 10)

WIND_ARROW_LEN = 4.0
WIND_BOX_ORIGIN_XY = cs.XY(6.0, 0.0)
WIND_BOX_Y_OFFSET = 0.5

GOAL_CHANGE_ROUND_DECIMALS = 3  # avoid float jitter spam

NO_GO_CONE_ARC_POINTS = 40  # number of points to approximate each cone arc
NO_GO_CONE_DEFAULT_RADIUS_KM = 1.5  # fallback radius when no axis range is available
NO_GO_CONE_RADIUS_FRACTION = 0.12  # fraction of the visible axis span to use as cone radius

OMPL_YAW_MARKER_SIZE = 16
OMPL_YAW_MARKER_COLOR = "darkviolet"

# A global waypoint is considered "reached" (and removed from the plot) once the boat comes within
# GLOBAL_WP_REACHED_KM of the local waypoint nearest to it, provided that local waypoint is itself
# within LOCAL_NEAR_GLOBAL_KM of the global waypoint.
GLOBAL_WP_REACHED_KM = 0.1
LOCAL_NEAR_GLOBAL_KM = 0.15
GLOBAL_WP_KEY_DECIMALS = 5  # lat/lon rounding used to identify a global waypoint across frames

# OpenStreetMap overlay (toggleable basemap drawn beneath the data traces).
OSM_TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
OSM_TILE_SIZE = 256  # px, standard slippy-map tile size
OSM_MAX_TILES_PER_AXIS = 4  # cap tiles fetched per axis to bound latency and figure payload size
OSM_MIN_ZOOM = 1
OSM_MAX_ZOOM = 18
OSM_REQUEST_TIMEOUT_S = 5
OSM_USER_AGENT = "UBCSailbot-LocalPathfinding-Visualizer"
OSM_OVERLAY_OPACITY = 1.0

# Module-level caches so the basemap is not refetched/restitched on every ~2.5 s frame.
_osm_tile_cache: Dict[Tuple[int, int, int], Any] = {}
_osm_overlay_cache: Dict[Any, Tuple[Any, Tuple[float, float, float, float]]] = {}


BASE_DIR = Path(__file__).resolve().parent
MOCK_NODES_DIR = BASE_DIR / "mock_nodes"
WIND_PARAMS_YAML = MOCK_NODES_DIR / "wind_params.yaml"
WIND_PARAMS_SH = MOCK_NODES_DIR / "wind_params.sh"

# Most recent VisualizerState, kept so view-only toggles (e.g. the map) can re-render the last
# frame immediately instead of waiting for the next ROS message.
_latest_vs: Optional["VisualizerState"] = None

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
        lat_deg: Latitude (degrees) of the AIS ship.
        lon_deg: Longitude (degrees) of the AIS ship.
        heading_deg: Heading (degrees) of the AIS ship.
        speed_kmph: Speed (km/h) of the AIS ship.
    """

    pos_x_km: float
    pos_y_km: float
    lat_deg: float
    lon_deg: float
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
        final_local_wp_headings_deg (List[float]): OMPL state yaws converted to navigation headings
                                                   (degrees) for the latest local path.

        sailbot_gps (List[ci.Gps]): GPS messages used for heading (degrees) and speed (km/h)
                                    display.

        ais_ships_by_id (Dict[int, AISShipData]): Dictionary mapping AIS ship IDs to their data
                                                   (position, heading, speed).

        land_obstacles_xy (List[Polygon]): Land obstacle polygons in XY (km).
        boat_obstacles_xy (List[Polygon]): Boat collision-zone polygons in XY (km).

        aw_vector_kmph (cs.XY): Apparent wind vector in global XY.
        tw_vector_kmph (cs.XY): True wind vector in global XY.
        bw_vector_kmph (cs.XY): Boat velocity vector in global XY.
        state_space (Optional[MultiPolygon]): The computed state-space overlay as a MultiPolygon.
    """

    def __init__(self, msgs: deque[ci.LPathData], last_replan_reason: str = ""):
        if not msgs:
            raise ValueError("VisualizerState requires at least one message")

        self.latest_msg = msgs[-1]
        self.last_replan_reason = last_replan_reason
        self._validate_message(self.latest_msg)

        # Boat history
        self.sailbot_lat_lon = [msg.gps.lat_lon for msg in msgs]
        self.sailbot_gps = [msg.gps for msg in msgs]

        # Paths
        self.all_local_wp = [msg.local_path.waypoints for msg in msgs]
        self.global_path = self.latest_msg.global_path

        # XY frame is anchored to the global path's final waypoint (the voyage destination).
        self.reference_lat_lon = self.global_path.waypoints[-1]

        # Global waypoints (lat/lon and XY) for plotting and reached-tracking
        self.global_wp = list(self.global_path.waypoints)
        self.global_wp_xy = cs.latlon_list_to_xy_list(self.reference_lat_lon, self.global_wp)

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
        self.final_local_wp_headings_deg = [
            waypoint.heading.heading for waypoint in self.all_local_wp[-1]
        ]
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
                pos_x_km=x,
                pos_y_km=y,
                lat_deg=ship.lat_lon.latitude,
                lon_deg=ship.lat_lon.longitude,
                heading_deg=ship.cog.heading,
                speed_kmph=ship.sog.speed,
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
        tw_dir_deg_gc, tw_speed_kmph = wcs.aw_gc_to_tw_gc(
            aw_dir_global_deg, aw_speed_kmph, boat_heading_deg, boat_speed_kmph
        )
        tw_dir_rad_gc = math.radians(tw_dir_deg_gc)
        self.tw_vector_kmph = cs.polar_to_cartesian(tw_dir_rad_gc, tw_speed_kmph)

        # Boat wind vector
        boat_wind_radians = math.radians(cs.bound_to_180(boat_heading_deg + 180))
        self.bw_vector_kmph = cs.polar_to_cartesian(boat_wind_radians, boat_speed_kmph)

        # State space
        self.state_space: Optional[MultiPolygon] = None

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


def global_wp_key(waypoint: ci.HelperLatLon) -> str:
    """Return a stable, JSON-serializable identifier for a global waypoint.

    Global waypoints are identified by their rounded lat/lon (rather than list index) so the
    reached-set persisted in the browser stays correct even if the global path is reordered.

    Args:
        waypoint: Global waypoint as a lat/lon.

    Returns:
        A string key of the form "lat,lon" rounded to GLOBAL_WP_KEY_DECIMALS.
    """
    lat = round(waypoint.latitude, GLOBAL_WP_KEY_DECIMALS)
    lon = round(waypoint.longitude, GLOBAL_WP_KEY_DECIMALS)
    return f"{lat},{lon}"


def compute_reached_global_wps(
    vs: VisualizerState,
    boat_xy_km: Tuple[float, float],
    local_x_km: List[float],
    local_y_km: List[float],
    reached_keys: Optional[List[str]],
) -> List[str]:
    """Update the set of global waypoints the boat has reached.

    A global waypoint is marked reached once the local waypoint nearest to it (within
    LOCAL_NEAR_GLOBAL_KM) has been reached by the boat (boat within GLOBAL_WP_REACHED_KM of that
    local waypoint). Once reached, a waypoint stays reached — the set only grows — so a waypoint
    does not reappear if the boat later drifts away.

    Args:
        vs: VisualizerState containing the global waypoints in XY.
        boat_xy_km: (x, y) current boat position in km.
        local_x_km: Local path X coordinates in km.
        local_y_km: Local path Y coordinates in km.
        reached_keys: Previously reached global waypoint keys, or None on first frame.

    Returns:
        The updated list of reached global waypoint keys.
    """
    reached = set(reached_keys or [])
    if not local_x_km or not local_y_km:
        return list(reached)

    bx, by = boat_xy_km[0], boat_xy_km[1]
    for waypoint, wp_xy in zip(vs.global_wp, vs.global_wp_xy):
        key = global_wp_key(waypoint)
        if key in reached:
            continue

        # nearest local waypoint to this global waypoint
        nearest_lx, nearest_ly, nearest_dist = min(
            (
                (lx, ly, math.hypot(lx - wp_xy.x, ly - wp_xy.y))
                for lx, ly in zip(local_x_km, local_y_km)
            ),
            key=lambda t: t[2],
        )

        boat_to_local = math.hypot(bx - nearest_lx, by - nearest_ly)
        if nearest_dist <= LOCAL_NEAR_GLOBAL_KM and boat_to_local <= GLOBAL_WP_REACHED_KM:
            reached.add(key)

    return list(reached)


def build_global_wp_trace(vs: VisualizerState, reached_keys: Optional[List[str]]) -> go.Scatter:
    """Build the marker trace for the (not-yet-reached) global waypoints.

    Args:
        vs: VisualizerState containing the global waypoints (lat/lon and XY).
        reached_keys: Keys of global waypoints already reached by the boat; these are omitted.

    Returns:
        A Plotly Scatter trace of the remaining global waypoints. Empty if all are reached.
    """
    reached = set(reached_keys or [])
    xs: List[float] = []
    ys: List[float] = []
    labels: List[str] = []
    customdata: List[List[float]] = []

    # The last global waypoints is the starting point and the first waypoint is the destination
    n = len(vs.global_wp)
    for i, (waypoint, wp_xy) in enumerate(zip(vs.global_wp, vs.global_wp_xy)):
        if global_wp_key(waypoint) in reached:
            continue
        xs.append(wp_xy.x)
        ys.append(wp_xy.y)
        labels.append(f"GW{n-i}")
        customdata.append([waypoint.latitude, waypoint.longitude])

    return go.Scatter(
        x=xs,
        y=ys,
        mode="markers+text",
        marker=dict(symbol="star", color="green", size=14, line=dict(width=1, color="darkgreen")),
        text=labels,
        textposition="bottom center",
        name="Global Waypoint",
        customdata=customdata,
        hovertemplate=(
            "<b>🌐 Global Waypoint</b><br>"
            "Lat: %{customdata[0]:.6f}°<br>"
            "Lon: %{customdata[1]:.6f}°<br>"
            "X: %{x:.2f}<br>"
            "Y: %{y:.2f}<br>"
            "<extra></extra>"
        ),
    )


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


def build_intermediate_trace(
    local_x_km: List[float], local_y_km: List[float], reference: ci.HelperLatLon
) -> go.Scatter:
    """
    Create the scatter trace for intermediate local waypoints (excluding start and goal).

    Args:
        local_x_km: X coordinates of the local waypoint list (km).
        local_y_km: Y coordinates of the local waypoint list (km).
        reference: Lat/Lon anchor of the XY frame (the global path's final waypoint), used to
            convert each waypoint's XY back to a true lat/lon for the hover text.

    Returns:
        A Plotly Scatter trace containing marker with text labels for intermediate waypoints.
        If fewer than 3 points exist, returns an empty trace (i.e., no intermediate points).
    """
    if len(local_x_km) < 3:
        return go.Scatter(x=[], y=[], mode="markers+text", name="Intermediate")
    labels = [f"LW{i+1}" for i, _ in enumerate(local_x_km[1:-1])]
    latlons = [
        cs.xy_to_latlon(reference, cs.XY(x, y)) for x, y in zip(local_x_km[1:-1], local_y_km[1:-1])
    ]
    customdata = [[ll.latitude, ll.longitude] for ll in latlons]
    return go.Scatter(
        x=local_x_km[1:-1],
        y=local_y_km[1:-1],
        mode="markers+text",
        marker=dict(color="blue", size=8),
        text=labels,
        textposition="top center",
        name="Intermediate",
        customdata=customdata,
        hovertemplate=(
            "Lat: %{customdata[0]:.6f}°<br>"
            "Lon: %{customdata[1]:.6f}°<br>"
            "X: %{x:.2f}<br>"
            "Y: %{y:.2f}<br>"
            "<extra></extra>"
        ),
    )


def build_goal_trace(
    goal_xy_km: Tuple[float, float], angle_deg: float, reference: ci.HelperLatLon
) -> go.Scatter:
    """
    Create the marker trace for the local goal waypoint.

    Args:
        goal_xy_km: (x, y) goal position in km.
        angle_deg: Angle from boat to goal in degrees (visualizer convention) used in hover text.
        reference: Lat/Lon anchor of the XY frame (the global path's final waypoint), used to
            convert the goal's XY back to a true lat/lon for the hover text.

    Returns:
        A Plotly Scatter trace representing the goal point.
    """
    goal_latlon = cs.xy_to_latlon(reference, cs.XY(goal_xy_km[0], goal_xy_km[1]))
    return go.Scatter(
        x=[goal_xy_km[0]],
        y=[goal_xy_km[1]],
        mode="markers",
        marker=dict(color="red", size=10),
        name="Goal",
        hovertemplate=f"Lat: {goal_latlon.latitude:.6f}° <br>"
        + f"Lon: {goal_latlon.longitude:.6f}° <br>"
        + "X: %{x:.2f} <br>"
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
        x=[boat_xy_km[0]] + list(local_x_km[1:]),
        y=[boat_xy_km[1]] + list(local_y_km[1:]),
        mode="lines",
        name="Path to Goal",
        line=dict(width=2, dash="dot", color="blue"),
        hovertemplate="X: %{x:.2f}<br>Y: %{y:.2f}<extra></extra>",
    )


def build_ompl_yaw_trace(
    local_x_km: List[float],
    local_y_km: List[float],
    headings_deg: List[float],
) -> go.Scatter:
    """Draw OMPL's yaw at every state in the latest local path.

    The path message stores each OMPL yaw as a navigation heading in degrees (0 degrees is north,
    increasing clockwise). Plotly arrow markers use the same visual convention when ``angleref``
    is ``"up"``. The original OMPL Cartesian yaw is reconstructed for hover text.

    Args:
        local_x_km: Local path X coordinates in km.
        local_y_km: Local path Y coordinates in km.
        headings_deg: OMPL yaws converted to navigation headings in degrees.

    Returns:
        A Plotly arrow-marker trace with one marker per OMPL path state.

    Raises:
        ValueError: If the coordinate and heading arrays have different lengths.
    """
    if not local_x_km and not local_y_km and not headings_deg:
        return go.Scatter(x=[], y=[], mode="markers", name="OMPL Yaw")

    if not (len(local_x_km) == len(local_y_km) == len(headings_deg)):
        raise ValueError("OMPL yaw visualization data must have matching lengths")

    ompl_yaws_rad = [
        math.atan2(
            math.sin(math.radians(cs.true_bearing_to_OMPL_cartesian(heading))),
            math.cos(math.radians(cs.true_bearing_to_OMPL_cartesian(heading))),
        )
        for heading in headings_deg
    ]
    customdata = [
        [index, heading, yaw]
        for index, (heading, yaw) in enumerate(zip(headings_deg, ompl_yaws_rad))
    ]

    return go.Scatter(
        x=local_x_km,
        y=local_y_km,
        mode="markers",
        name="OMPL Yaw",
        customdata=customdata,
        hovertemplate=(
            "<b>OMPL state %{customdata[0]}</b><br>"
            "Heading: %{customdata[1]:.1f} degrees<br>"
            "OMPL yaw: %{customdata[2]:.3f} rad<br>"
            "X: %{x:.2f} km<br>"
            "Y: %{y:.2f} km"
            "<extra></extra>"
        ),
        marker=dict(
            symbol="arrow",
            color=OMPL_YAW_MARKER_COLOR,
            line=dict(width=1, color="white"),
            size=OMPL_YAW_MARKER_SIZE,
            angleref="up",
            angle=[cs.true_bearing_to_plotly_cartesian(heading) for heading in headings_deg],
        ),
    )


def build_boat_trace(
    vs: VisualizerState, boat_xy_km: Tuple[float, float], dist_to_goal_km: float
) -> go.Scatter:
    """
    Create the boat marker trace (filled arrow-head/ triangle) at the current boat position.

    Args:
        vs: VisualizerState containing GPS heading/speed history.
        boat_xy_km: (x, y) current boat position in km.
        dist_to_goal_km: Current straight-line distance to the goal in km (for hover text).

    Returns:
        A Plotly Scatter trace representing the boat marker with heading-based rotation.
    """
    heading = vs.sailbot_gps[-1].heading.heading
    speed = vs.sailbot_gps[-1].speed.speed
    lat = vs.sailbot_lat_lon[-1].latitude
    lon = vs.sailbot_lat_lon[-1].longitude
    return go.Scatter(
        x=[boat_xy_km[0]],
        y=[boat_xy_km[1]],
        mode="markers",
        name="Boat",
        hovertemplate=(
            "<b>🚢 Sailbot Current Position</b><br>"
            f"Lat: {lat:.6f}°<br>"
            f"Lon: {lon:.6f}°<br>"
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


def _compute_cone_radius(last_range: Optional[Dict[str, List[float]]]) -> float:
    """Pick a cone radius that looks proportional to the current view.

    If the caller has stored axis ranges from a previous render, the radius is
    a fixed fraction of the smaller visible span so the cones scale with zoom.
    Otherwise the default constant is returned.

    Args:
        last_range: Previously stored axis ranges {"x": [lo, hi], "y": [lo, hi]} or None.

    Returns:
        Radius in km for the no-go zone cone arcs.
    """
    if last_range is not None:
        x_span = abs(last_range["x"][1] - last_range["x"][0])
        y_span = abs(last_range["y"][1] - last_range["y"][0])
        return max(min(x_span, y_span) * NO_GO_CONE_RADIUS_FRACTION, 0.2)
    return NO_GO_CONE_DEFAULT_RADIUS_KM


def build_no_go_zone_traces(
    vs: VisualizerState,
    boat_xy_km: Tuple[float, float],
    last_range: Optional[Dict[str, List[float]]] = None,
) -> List[go.Scatter]:
    """Build semi-transparent cone traces showing the upwind and downwind no-go zones.

    Each cone is a circular sector (pie-slice) centred on the boat and spanning
    ±NO_GO_ZONE (45°) either side of the upwind / downwind direction.  The cones
    are drawn as filled Scatter polygons so they integrate with the existing
    Plotly figure.

    The true-wind direction is recovered from the Cartesian vector stored in
    *vs.tw_vector_kmph* — if the wind speed is effectively zero the cones are
    not drawn (there is no meaningful wind direction).

    Args:
        vs: VisualizerState containing the true-wind vector.
        boat_xy_km: (x, y) current boat position in km.
        last_range: Previously stored axis ranges, used to auto-scale the cone
                    radius to the current zoom level.

    Returns:
        List of Plotly Scatter traces (0, 1, or 2 cones).
    """
    tw_mag = math.hypot(vs.tw_vector_kmph.x, vs.tw_vector_kmph.y)
    if tw_mag < 1e-6:
        return []  # no meaningful wind direction

    # True wind bearing (true-bearing convention: 0 = north, +π/2 = east)
    tw_dir_rad = math.atan2(vs.tw_vector_kmph.x, vs.tw_vector_kmph.y)

    # Upwind centre is opposite to the wind vector; downwind is with it
    upwind_centre_rad = tw_dir_rad + math.pi
    downwind_centre_rad = tw_dir_rad

    radius = _compute_cone_radius(last_range)
    bx, by = boat_xy_km[0], boat_xy_km[1]

    def _make_sector(centre_rad: float) -> Tuple[List[float], List[float]]:
        """Return (xs, ys) for a filled pie-slice polygon."""
        xs = [bx]  # start at the boat
        ys = [by]
        start_angle = centre_rad - NO_GO_ZONE
        end_angle = centre_rad + NO_GO_ZONE
        for i in range(NO_GO_CONE_ARC_POINTS + 1):
            theta = start_angle + (end_angle - start_angle) * i / NO_GO_CONE_ARC_POINTS
            # polar_to_cartesian convention: x = sin(θ), y = cos(θ)
            xs.append(bx + radius * math.sin(theta))
            ys.append(by + radius * math.cos(theta))
        xs.append(bx)  # close back to the boat
        ys.append(by)
        return xs, ys

    traces: List[go.Scatter] = []

    # Upwind no-go cone
    ux, uy = _make_sector(upwind_centre_rad)
    traces.append(
        go.Scatter(
            x=ux,
            y=uy,
            fill="toself",
            mode="lines",
            line=dict(color="rgba(255, 80, 80, 0.6)", width=1),
            fillcolor="rgba(255, 80, 80, 0.18)",
            opacity=1.0,
            name="Upwind No-Go",
            hoverinfo="skip",
            showlegend=True,
        )
    )

    # Downwind no-go cone
    dx, dy = _make_sector(downwind_centre_rad)
    traces.append(
        go.Scatter(
            x=dx,
            y=dy,
            fill="toself",
            mode="lines",
            line=dict(color="rgba(255, 165, 0, 0.6)", width=1),
            fillcolor="rgba(255, 165, 0, 0.18)",
            opacity=1.0,
            name="Downwind No-Go",
            hoverinfo="skip",
            showlegend=True,
        )
    )

    return traces


def build_ais_traces(vs: VisualizerState) -> List[go.Scatter]:
    """
    Build AIS ship markers (filled arrow-heads/ triangles) for the plot.

    Args:
        vs: VisualizerState containing AIS ship data by ID.

    Returns:
        List of Plotly Scatter traces for AIS ships.
    """
    traces = []
    for ais_id, ship_data in vs.ais_ships_by_id.items():
        x_val = ship_data.pos_x_km
        y_val = ship_data.pos_y_km
        lat = ship_data.lat_deg
        lon = ship_data.lon_deg
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
                    f"Lat: {lat:.6f}°<br>"
                    f"Lon: {lon:.6f}°<br>"
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


def configure_wind_box_elements(vs: VisualizerState) -> WindBoxConfig:
    """
    Build the "wind box" inset configuration with scaled wind and boat velocity vectors.

    Constructs all components needed to render the wind box inset, including:
    - Secondary axes configuration (x2/y2) for the inset domain and range.
    - Normalized/scaled arrow annotations for apparent wind, true wind, and boat velocity.
    - Background rectangle shape for the inset.
    - Text annotations for wind vector labels and speed values.

    Args:
        vs: VisualizerState containing wind vectors in global XY frame.

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
    aw_unit = get_unit_vector(vs.aw_vector_kmph)
    tw_unit = get_unit_vector(vs.tw_vector_kmph)
    bw_unit = get_unit_vector(vs.bw_vector_kmph)

    aw_mag = math.hypot(vs.aw_vector_kmph.x, vs.aw_vector_kmph.y)
    tw_mag = math.hypot(vs.tw_vector_kmph.x, vs.tw_vector_kmph.y)
    bw_mag = math.hypot(vs.bw_vector_kmph.x, vs.bw_vector_kmph.y)

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
    vs: VisualizerState,
    boat_xy_km: Tuple[float, float],
    goal_xy_km: Tuple[float, float],
    fig: go.Figure,
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

    vs.state_space = state_space

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


def apply_layout(
    vs: VisualizerState,
    fig: go.Figure,
    zoom_needed: bool,
    last_range: Optional[Dict[str, List[float]]],
    show_map: bool = False,
) -> None:
    """
    Apply the main plot layout configuration (axis titles, domains, legend, and optional ranges).

    Args:
        fig: Target Plotly figure.
        zoom_needed: whether we want to zoom into the state space
        last_range: previously stored axis ranges to maintain axes if zoom not needed.
        show_map: when True, constrain the axes to equal scale so the OpenStreetMap overlay stays
            geographically proportioned instead of stretched. Left unconstrained otherwise to
            preserve the default plot behavior.
    """
    xaxis: Dict[str, Any] = dict(domain=[0.0, 0.98])
    yaxis: Dict[str, Any] = dict(domain=[0.30, 1.0])
    if show_map:
        # scaleanchor/scaleratio keep 1 km on X the same on-screen size as 1 km on Y so the
        # basemap is not distorted. Only applied with the map on, to keep the default look.
        yaxis.update(scaleanchor="x", scaleratio=1.0)

    # Base Layout
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

    # Behavior for zooming into state space / persisting user changes
    if zoom_needed:
        min_bounds, max_bounds = get_state_space_bounds(vs)
        fig.update_layout(
            xaxis=dict(range=[min_bounds.x, max_bounds.x], autorange=False),
            yaxis=dict(range=[min_bounds.y, max_bounds.y], autorange=False),
        )
    elif last_range is not None:
        fig.update_layout(
            xaxis=dict(range=last_range["x"], autorange=False),
            yaxis=dict(range=last_range["y"], autorange=False),
        )


def build_figure(
    vs: VisualizerState,
    last_goal_xy_km: Optional[Tuple[float, float]],
    last_range: Optional[Dict[str, List[float]]],
    reached_global_keys: Optional[List[str]] = None,
    show_map: bool = False,
) -> Tuple[go.Figure, Tuple[float, float], List[str]]:
    """
    Builds and renders the complete path planning visualization figure.

    This function orchestrates all visual elements: boat state, local path, obstacles,
    AIS ships, wind vectors, and state-space overlay.

    Args:
        vs: VisualizerState containing processed ROS message data (boat position, path,
               obstacles, AIS ships, wind vectors).
        last_goal_xy_km: Previous goal position (x, y) in km, or None on first render. Used to
                      detect goal changes and show a popup message.
        reached_global_keys: Keys of global waypoints already reached by the boat (persisted
                      across frames); reached waypoints are not plotted.
        show_map: Whether to draw the OpenStreetMap basemap beneath the data.

    Returns:
       (fig, new_goal_xy_rounded, reached_global_keys):
            - fig: Updated Plotly figure ready for display.
            - new_goal_xy_rounded: Current goal position rounded to GOAL_CHANGE_ROUND_DECIMALS
                                for float jitter tolerance.

    Raises:
        ValueError: If no local waypoints are available for plotting.
    """
    fig = initial_plot()

    local_x_km = list(vs.final_local_wp_x_km)
    local_y_km = list(vs.final_local_wp_y_km)

    # Boat and goal info
    boat_xy_km = cs.XY(vs.sailbot_pos_x_km[-1], vs.sailbot_pos_y_km[-1])

    if not local_x_km or not local_y_km:
        # No local path available (e.g. pathfinding failed and sail was disabled).
        # Still render the boat, obstacles, AIS traffic, and wind so the operator keeps
        # situational awareness instead of seeing a blank or crashed view.
        return build_figure_without_local_path(
            vs, boat_xy_km, last_goal_xy_km, last_range, reached_global_keys, show_map
        )
    goal_xy_km = cs.XY(local_x_km[-1], local_y_km[-1])
    goal_change = compute_goal_change(last_goal_xy_km, goal_xy_km)

    # Update which global waypoints the boat has reached, then plot the remaining ones
    reached_global_keys = compute_reached_global_wps(
        vs, boat_xy_km, local_x_km, local_y_km, reached_global_keys
    )

    # Computing angle and distance from boat to goal
    angle_deg = math.degrees(
        math.atan2(goal_xy_km[0] - boat_xy_km[0], goal_xy_km[1] - boat_xy_km[1])
    )
    dist_km = math.hypot(goal_xy_km[0] - boat_xy_km[0], goal_xy_km[1] - boat_xy_km[1])

    # adding all the Traces(global, intermediate, goal, boat and path) to the plot
    fig.add_trace(build_global_wp_trace(vs, reached_global_keys))
    fig.add_trace(build_intermediate_trace(local_x_km, local_y_km, vs.reference_lat_lon))
    fig.add_trace(build_goal_trace(goal_xy_km, angle_deg, vs.reference_lat_lon))
    fig.add_trace(build_boat_trace(vs, boat_xy_km, dist_km))
    path_trace = build_path_trace(local_x_km, local_y_km, boat_xy_km)
    if path_trace is not None:
        fig.add_trace(path_trace)
    fig.add_trace(
        build_ompl_yaw_trace(local_x_km, local_y_km, vs.final_local_wp_headings_deg)
    )

    # Adding Obstacle (both Land and Boat) and AIS ships to the plot
    land_traces = build_polygon_traces(
        vs.land_obstacles_xy,
        name="Land Obstacle",
        line={"color": "lightgreen"},
        fillcolor="lightgreen",
        opacity=0.5,
        showlegend=True,
    )
    boat_traces = build_polygon_traces(
        vs.boat_obstacles_xy,
        name="AIS Collision Zone",
        line={"width": 2},
        fillcolor="rgba(255,165,0,0.25)",
        opacity=0.5,
        hoverinfo="skip",
        showlegend=False,
    )
    ais_traces = build_ais_traces(vs)
    no_go_traces = build_no_go_zone_traces(vs, boat_xy_km, last_range)
    fig.add_traces(land_traces + boat_traces + ais_traces + no_go_traces)

    # Creating Wind box and its elements and adding them to the plot
    wind_config = configure_wind_box_elements(vs)
    fig.update_layout(**wind_config.layout_config)
    for arrow in wind_config.wind_arrows:
        fig.add_annotation(arrow)
    fig.add_shape(**wind_config.background_info)
    for annotation in wind_config.annotations:
        fig.add_annotation(annotation)

    # Computing State space overlay and adding it to the plot
    zoom_needed = last_range is None
    compute_and_add_state_space(vs, boat_xy_km, goal_xy_km, fig)
    apply_layout(vs, fig, zoom_needed, last_range, show_map)
    add_goal_change_popup(fig, goal_change.message)  # Popup message for goal change

    if show_map:
        add_map_overlay(fig, vs, last_range)

    return fig, goal_change.new_goal_xy_rounded, reached_global_keys


def build_figure_without_local_path(
    vs: VisualizerState,
    boat_xy_km: Tuple[float, float],
    last_goal_xy_km: Optional[Tuple[float, float]],
    last_range: Optional[Dict[str, List[float]]],
    reached_global_keys: Optional[List[str]] = None,
    show_map: bool = False,
) -> Tuple[go.Figure, Tuple[float, float], List[str]]:
    """
    Build the visualization when no local path is available.

    This happens when local pathfinding fails (e.g. the solver could not produce a path and
    sail was disabled), so there are no local waypoints or local goal to plot. Everything that
    does not depend on the local path is still drawn — the boat, global waypoints, land/AIS
    obstacles, AIS ships, and the wind box — so the operator retains situational awareness
    instead of seeing a blank or crashed figure.

    Without a local path the reached-set cannot be updated (there is no local waypoint to test
    against), so the existing reached-set is passed through unchanged.

    Args:
        vs: VisualizerState containing processed ROS message data.
        boat_xy_km: (x, y) current boat position in km.
        last_goal_xy_km: Previous goal position (x, y) in km, or None if never set.
        last_range: Previously stored axis ranges, or None on first render.
        reached_global_keys: Keys of global waypoints already reached; passed through unchanged.
        show_map: Whether to draw the OpenStreetMap basemap beneath the data.
    Returns:
        (fig, goal_xy, reached_global_keys): The figure, the goal position to persist downstream
        (the previous goal if known, otherwise the current boat position so goal-change tracking
        stays consistent), and the unchanged reached-set.
    """
    fig = initial_plot()
    reached_global_keys = list(reached_global_keys or [])

    # Boat marker (distance-to-goal is unknown without a local path)
    fig.add_trace(build_boat_trace(vs, boat_xy_km, dist_to_goal_km=0.0))

    # Global waypoints (still useful for situational awareness when the local path is missing)
    fig.add_trace(build_global_wp_trace(vs, reached_global_keys))

    # Obstacles (land and AIS collision zones) and AIS ships
    land_traces = build_polygon_traces(
        vs.land_obstacles_xy,
        name="Land Obstacle",
        line={"color": "lightgreen"},
        fillcolor="lightgreen",
        opacity=0.5,
        showlegend=True,
    )
    boat_traces = build_polygon_traces(
        vs.boat_obstacles_xy,
        name="AIS Collision Zone",
        line={"width": 2},
        fillcolor="rgba(255,165,0,0.25)",
        opacity=0.5,
        hoverinfo="skip",
        showlegend=False,
    )
    ais_traces = build_ais_traces(vs)
    no_go_traces = build_no_go_zone_traces(vs, boat_xy_km, last_range)
    fig.add_traces(land_traces + boat_traces + ais_traces + no_go_traces)

    # Wind box and its elements
    wind_config = configure_wind_box_elements(vs)
    fig.update_layout(**wind_config.layout_config)
    for arrow in wind_config.wind_arrows:
        fig.add_annotation(arrow)
    fig.add_shape(**wind_config.background_info)
    for annotation in wind_config.annotations:
        fig.add_annotation(annotation)

    # State-space overlay around the boat only (there is no goal to include)
    boat_box = OMPLPath.create_buffer_around_position(
        cs.XY(boat_xy_km[0], boat_xy_km[1]), BOX_BUFFER_SIZE_KM
    )
    vs.state_space = MultiPolygon([boat_box])
    x_min, y_min, x_max, y_max = vs.state_space.bounds
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

    # Make it obvious why no path is drawn
    fig.add_annotation(
        text="⚠️ No local path available (sail disabled)",
        xref="paper",
        yref="paper",
        x=0.5,
        y=1.0,
        showarrow=False,
        font=dict(color="red", size=16),
        bgcolor="rgba(255, 255, 255, 0.7)",
    )

    zoom_needed = last_range is None
    apply_layout(vs, fig, zoom_needed, last_range, show_map)

    if show_map:
        add_map_overlay(fig, vs, last_range)

    goal_xy = last_goal_xy_km if last_goal_xy_km is not None else (boat_xy_km[0], boat_xy_km[1])
    return fig, goal_xy, reached_global_keys


def write_wind_params(tw_dir_deg: float, tw_speed_kmph: float) -> None:
    with open(WIND_PARAMS_YAML, "r") as f:
        data = yaml.safe_load(f)

    data["/mock_wind_sensor"]["ros__parameters"]["tw_dir_deg"] = tw_dir_deg
    data["/mock_wind_sensor"]["ros__parameters"]["tw_speed_kmph"] = float(tw_speed_kmph)

    data["/mock_gps"]["ros__parameters"]["tw_dir_deg"] = tw_dir_deg
    data["/mock_gps"]["ros__parameters"]["tw_speed_kmph"] = float(tw_speed_kmph)

    with open(WIND_PARAMS_YAML, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    apply_wind_params()


def apply_wind_params():
    subprocess.run(
        ["bash", str(WIND_PARAMS_SH)],
        check=True,
    )


def apply_gps_params(
    use_gps_noise: bool,
    use_ocean_drift: bool,
    use_drift_randomization: bool,
    drift_speed_kmph: float,
    drift_dir_deg: float,
    drift_accel_kmph2: float,
) -> None:
    """Apply GPS simulation parameters to the live mock_gps ROS node via ros2 param set."""
    params = [
        ("use_gps_noise", str(use_gps_noise).lower()),
        ("use_ocean_drift", str(use_ocean_drift).lower()),
        ("use_drift_randomization", str(use_drift_randomization).lower()),
        ("ocean_drift_speed_kmph", str(float(drift_speed_kmph))),
        ("ocean_drift_dir_deg", str(float(drift_dir_deg))),
        ("ocean_drift_accel_kmph2", str(float(drift_accel_kmph2))),
    ]
    for param_name, value in params:
        subprocess.run(
            ["ros2", "param", "set", "/mock_gps", param_name, value],
            check=True,
            capture_output=True,
        )


def get_state_space_bounds(
    vs: VisualizerState,
) -> Tuple[cs.XY, cs.XY]:
    """
    Gets the state space bounds as min and max XY coordinates.

    Args:
        vs: VisualizerState containing the current state space.

    Returns:
        A tuple of (min_xy, max_xy) representing the state space bounds.
        Falls back to DEFAULT_PLOT_RANGE if state space is not available.
    """
    if vs.state_space is None:
        return (
            cs.XY(DEFAULT_PLOT_RANGE[0], DEFAULT_PLOT_RANGE[0]),
            cs.XY(DEFAULT_PLOT_RANGE[1], DEFAULT_PLOT_RANGE[1]),
        )

    x_min, y_min, x_max, y_max = vs.state_space.bounds
    return (
        cs.XY(x_min, y_min),
        cs.XY(x_max, y_max),
    )


# --------------------------------------
# OpenStreetMap basemap overlay
# --------------------------------------
def _deg2num(lat_deg: float, lon_deg: float, zoom: int) -> Tuple[float, float]:
    """Convert lat/lon to fractional slippy-map tile coordinates at the given zoom."""
    lat_rad = math.radians(lat_deg)
    n = 2.0**zoom
    xt = (lon_deg + 180.0) / 360.0 * n
    yt = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return xt, yt


def _num2deg(xt: float, yt: float, zoom: int) -> Tuple[float, float]:
    """Convert fractional slippy-map tile coordinates to the lat/lon of that tile's NW corner."""
    n = 2.0**zoom
    lon = xt / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * yt / n))))
    return lat, lon


def _choose_osm_zoom(lat_n: float, lat_s: float, lon_w: float, lon_e: float) -> int:
    """Pick the highest zoom whose tile span fits within OSM_MAX_TILES_PER_AXIS on both axes."""
    for zoom in range(OSM_MAX_ZOOM, OSM_MIN_ZOOM - 1, -1):
        x0 = math.floor(_deg2num(lat_n, lon_w, zoom)[0])
        x1 = math.floor(_deg2num(lat_s, lon_e, zoom)[0])
        y0 = math.floor(_deg2num(lat_n, lon_w, zoom)[1])
        y1 = math.floor(_deg2num(lat_s, lon_e, zoom)[1])
        if (x1 - x0 + 1) <= OSM_MAX_TILES_PER_AXIS and (y1 - y0 + 1) <= OSM_MAX_TILES_PER_AXIS:
            return zoom
    return OSM_MIN_ZOOM


def _fetch_osm_tile(zoom: int, x: int, y: int):
    """Fetch a single OSM tile (RGB), caching it in-process to respect the tile usage policy."""
    key = (zoom, x, y)
    if key in _osm_tile_cache:
        return _osm_tile_cache[key]
    url = OSM_TILE_URL.format(z=zoom, x=x, y=y)
    resp = requests.get(url, headers={"User-Agent": OSM_USER_AGENT}, timeout=OSM_REQUEST_TIMEOUT_S)
    resp.raise_for_status()
    tile = Image.open(BytesIO(resp.content)).convert("RGB")
    _osm_tile_cache[key] = tile
    return tile


def _build_osm_image(
    lat_n: float, lat_s: float, lon_w: float, lon_e: float
) -> Tuple[Any, Tuple[float, float, float, float]]:
    """Stitch the OSM tiles covering a lat/lon bbox into one image.

    Args:
        lat_n, lat_s: North/south latitude bounds of the desired view.
        lon_w, lon_e: West/east longitude bounds of the desired view.

    Returns:
        (image, (lat_top, lat_bot, lon_left, lon_right)) where the second element is the actual
        geographic extent of the stitched mosaic (tile-aligned, so slightly larger than the bbox).
    """
    zoom = _choose_osm_zoom(lat_n, lat_s, lon_w, lon_e)
    x0 = math.floor(_deg2num(lat_n, lon_w, zoom)[0])
    x1 = math.floor(_deg2num(lat_s, lon_e, zoom)[0])
    y0 = math.floor(_deg2num(lat_n, lon_w, zoom)[1])
    y1 = math.floor(_deg2num(lat_s, lon_e, zoom)[1])

    mosaic = Image.new("RGB", ((x1 - x0 + 1) * OSM_TILE_SIZE, (y1 - y0 + 1) * OSM_TILE_SIZE))
    for xi in range(x0, x1 + 1):
        for yi in range(y0, y1 + 1):
            tile = _fetch_osm_tile(zoom, xi, yi)
            mosaic.paste(tile, ((xi - x0) * OSM_TILE_SIZE, (yi - y0) * OSM_TILE_SIZE))

    lat_top, lon_left = _num2deg(x0, y0, zoom)
    lat_bot, lon_right = _num2deg(x1 + 1, y1 + 1, zoom)
    return mosaic, (lat_top, lat_bot, lon_left, lon_right)


def _view_bounds_xy(
    vs: VisualizerState, last_range: Optional[Dict[str, List[float]]]
) -> Tuple[float, float, float, float]:
    """Return the (x_min, x_max, y_min, y_max) of the currently displayed view in km."""
    if last_range is not None:
        return (last_range["x"][0], last_range["x"][1], last_range["y"][0], last_range["y"][1])
    min_b, max_b = get_state_space_bounds(vs)
    return (min_b.x, max_b.x, min_b.y, max_b.y)


def _add_map_message(fig: go.Figure, text: str) -> None:
    """Show a small bottom-right note on the figure (used for map attribution or errors)."""
    fig.add_annotation(
        text=text,
        xref="paper",
        yref="paper",
        x=0.98,  # right edge of the plot's x-axis domain
        y=0.30,  # bottom edge of the plot's y-axis domain
        xanchor="right",
        yanchor="bottom",
        showarrow=False,
        font=dict(color="black", size=9),
        bgcolor="rgba(255, 255, 255, 0.6)",
    )


def add_map_overlay(
    fig: go.Figure, vs: VisualizerState, last_range: Optional[Dict[str, List[float]]]
) -> None:
    """Draw an OpenStreetMap basemap beneath the data traces, georeferenced to the XY (km) frame.

    The visible view bounds (km) are converted to a lat/lon bbox via the goal-anchored reference,
    the covering OSM tiles are stitched into one image, and that image is placed as a layout image
    spanning the corresponding XY rectangle. Results are cached by view signature so the basemap is
    not refetched every frame. Any failure (missing deps, network error) is shown as a small
    annotation and otherwise ignored so the visualizer keeps working.

    Args:
        fig: Target Plotly figure.
        vs: VisualizerState (provides the reference lat/lon and state-space fallback bounds).
        last_range: Previously stored axis ranges, used as the view extent when available.
    """
    _add_map_message(fig, "Map unavailable (install requests and Pillow)")

    try:
        x_min, x_max, y_min, y_max = _view_bounds_xy(vs, last_range)
        ref = vs.reference_lat_lon

        # lat/lon bbox of the view's four XY corners
        corners = [
            cs.xy_to_latlon(ref, cs.XY(x, y)) for x in (x_min, x_max) for y in (y_min, y_max)
        ]
        lats = [c.latitude for c in corners]
        lons = [c.longitude for c in corners]
        lat_n, lat_s, lon_w, lon_e = max(lats), min(lats), min(lons), max(lons)

        sig = (
            round(ref.latitude, 4),
            round(ref.longitude, 4),
            round(lat_n, 4),
            round(lat_s, 4),
            round(lon_w, 4),
            round(lon_e, 4),
        )
        if sig in _osm_overlay_cache:
            mosaic, extent = _osm_overlay_cache[sig]
        else:
            mosaic, extent = _build_osm_image(lat_n, lat_s, lon_w, lon_e)
            if len(_osm_overlay_cache) > 16:
                _osm_overlay_cache.clear()
            _osm_overlay_cache[sig] = (mosaic, extent)

        lat_top, lat_bot, lon_left, lon_right = extent
        nw = cs.latlon_to_xy(ref, ci.HelperLatLon(latitude=lat_top, longitude=lon_left))
        se = cs.latlon_to_xy(ref, ci.HelperLatLon(latitude=lat_bot, longitude=lon_right))
        fig.add_layout_image(
            dict(
                source=mosaic,
                xref="x",
                yref="y",
                x=nw.x,
                y=nw.y,
                sizex=se.x - nw.x,
                sizey=nw.y - se.y,
                xanchor="left",
                yanchor="top",
                sizing="stretch",
                layer="below",
                opacity=OSM_OVERLAY_OPACITY,
            )
        )
    except Exception as e:
        # Network/tile/render failure: keep the rest of the figure usable.
        _add_map_message(fig, f"Map unavailable ({type(e).__name__})")
        return

    # OpenStreetMap tile usage policy requires visible attribution.
    _add_map_message(fig, "Map data from © OpenStreetMap")


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
            html.Div(
                id="path-status",
                children="Remaining Waypoints: --\nReplan Reason: --",
                style={
                    "position": "absolute",
                    "bottom": "235px",
                    "left": "50px",
                    "padding": "8px 16px",
                    "backgroundColor": "rgba(255, 255, 255, 0.88)",
                    "borderRadius": "8px",
                    "border": "1px solid #ccc",
                    "zIndex": "1000",
                    "fontFamily": "Consolas, monospace",
                    "fontSize": "13px",
                    "fontWeight": "bold",
                    "color": "rgb(18,70,139)",
                    "whiteSpace": "pre-line",
                },
            ),
            html.Div(
                id="control-panel",
                style={
                    "position": "absolute",
                    "bottom": "175px",
                    "left": "50px",  # Aligns with the Y-axis
                    "display": "flex",
                    "gap": "15px",
                    "alignItems": "center",
                    "padding": "10px 20px",
                    "backgroundColor": "rgba(255, 255, 255, 0.8)",  # Semi-transparent white
                    "borderRadius": "8px",
                    "border": "1px solid #ccc",
                    "zIndex": "1000",
                    "fontFamily": "Consolas, monospace",
                    "fontSize": "13px",
                },
                children=[
                    html.Span(
                        "Wind",
                        style={
                            "fontWeight": "bold",
                            "color": "rgb(18,70,139)",
                            "whiteSpace": "nowrap",
                        },
                    ),
                    html.Label("Wind Direction (°):", style={"fontWeight": "bold"}),
                    dcc.Input(id="tw-dir-input", type="number", value=0, style={"width": "80px"}),
                    html.Label("Wind Speed (km/h):", style={"fontWeight": "bold"}),
                    dcc.Input(
                        id="tw-speed-input", type="number", value=0, style={"width": "80px"}
                    ),
                    html.Button(
                        "Apply",
                        id="apply-wind-btn",
                        style={
                            "backgroundColor": "rgb(18, 70, 139)",
                            "color": "white",
                            "cursor": "pointer",
                        },
                    ),
                    html.Div(id="wind-status"),
                ],
            ),
            # ── GPS / Drift control panel ───────────────────────────────────────────
            html.Div(
                id="gps-control-panel",
                style={
                    "position": "absolute",
                    "bottom": "113px",
                    "left": "50px",
                    "display": "flex",
                    "flexWrap": "wrap",
                    "gap": "12px",
                    "alignItems": "center",
                    "padding": "10px 16px",
                    "backgroundColor": "rgba(255, 255, 255, 0.88)",
                    "borderRadius": "8px",
                    "border": "1px solid #ccc",
                    "zIndex": "1000",
                    "fontFamily": "Consolas, monospace",
                    "fontSize": "13px",
                },
                children=[
                    html.Span(
                        "GPS / Drift",
                        style={
                            "fontWeight": "bold",
                            "color": "rgb(18,70,139)",
                            "whiteSpace": "nowrap",
                        },
                    ),
                    dcc.Checklist(
                        id="gps-toggles",
                        options=[  # type: ignore[arg-type]
                            {"label": " GPS Noise", "value": "use_gps_noise"},
                            {"label": " Ocean Drift", "value": "use_ocean_drift"},
                            {"label": " Drift Randomization", "value": "use_drift_randomization"},
                        ],
                        value=["use_gps_noise", "use_ocean_drift", "use_drift_randomization"],
                        labelStyle={
                            "display": "inline-block",
                            "marginRight": "14px",
                            "cursor": "pointer",
                        },
                        style={"display": "flex", "alignItems": "center"},
                    ),
                    html.Label(
                        "Drift Speed (km/h):", style={"fontWeight": "bold", "whiteSpace": "nowrap"}
                    ),
                    dcc.Input(
                        id="drift-speed-input",
                        type="number",
                        value=0.5,
                        step=0.1,
                        min=0,
                        style={"width": "72px"},
                    ),
                    html.Label(
                        "Drift Dir (°):", style={"fontWeight": "bold", "whiteSpace": "nowrap"}
                    ),
                    dcc.Input(
                        id="drift-dir-input",
                        type="number",
                        value=45,
                        min=-180,
                        max=180,
                        style={"width": "72px"},
                    ),
                    html.Label(
                        "Drift Accel (km/h²):",
                        style={"fontWeight": "bold", "whiteSpace": "nowrap"},
                    ),
                    dcc.Input(
                        id="drift-accel-input",
                        type="number",
                        value=0.0,
                        step=0.1,
                        style={"width": "72px"},
                    ),
                    html.Button(
                        "Apply",
                        id="apply-gps-btn",
                        style={
                            "backgroundColor": "rgb(18, 70, 139)",
                            "color": "white",
                            "border": "none",
                            "borderRadius": "4px",
                            "padding": "5px 12px",
                            "cursor": "pointer",
                        },
                    ),
                    html.Div(
                        id="gps-status",
                        style={"color": "green", "fontSize": "12px", "minWidth": "160px"},
                    ),
                ],
            ),
            # ── Map control panel ───────────────────────────────────────────
            html.Div(
                id="Map-control-panel",
                style={
                    "position": "absolute",
                    # Stacked above the wind panel (175px); the GPS panel occupies 95px.
                    "bottom": "50px",
                    "left": "50px",
                    "display": "flex",
                    "flexWrap": "wrap",
                    "gap": "12px",
                    "alignItems": "center",
                    "padding": "10px 16px",
                    "backgroundColor": "rgba(255, 255, 255, 0.88)",
                    "borderRadius": "8px",
                    "border": "1px solid #ccc",
                    "zIndex": "1000",
                    "fontFamily": "Consolas, monospace",
                    "fontSize": "13px",
                },
                children=[
                    html.Span(
                        "Show Map",
                        style={
                            "fontWeight": "bold",
                            "color": "rgb(18,70,139)",
                            "whiteSpace": "nowrap",
                        },
                    ),
                    dcc.Checklist(
                        id="map-toggle",
                        options={"on": " Show map"},  # {value: label}
                        value=[],
                        style={"fontWeight": "bold", "marginLeft": "10px"},
                    ),
                ],
            ),
            dcc.Interval(id="interval-component", interval=UPDATE_INTERVAL_MS, n_intervals=0),
            dcc.Store(id="goal-store", data=None),
            dcc.Store(id="range-store", data=None),
            dcc.Store(id="reached-global-store", data=None),
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
    Output("reached-global-store", "data"),
    Output("path-status", "children"),
    Input("interval-component", "n_intervals"),
    Input("live-graph", "relayoutData"),
    Input("reset-button", "n_clicks"),
    Input("map-toggle", "value"),
    State("live-graph", "figure"),
    State("goal-store", "data"),
    State("range-store", "data"),
    State("reached-global-store", "data"),
    prevent_initial_call=True,
)
def update_graph(
    _: int,
    relayout_data,
    __: int,
    map_toggle: Optional[List[str]],
    current_figure,
    last_goal_xy_km: Optional[List[float]],
    stored_range,
    reached_global_keys: Optional[List[str]],
):
    """
    Dash callback: handles both interval updates and reset button clicks.
    Uses callback_context to determine which input triggered the update.

    Args:
        _: Dash interval tick (unused).
        relayout_data: Data relayed from Plotly when user pans, zooms in/out, autoscales
        __: Reset button n_clicks (unused).
        map_toggle: Value of the "Show map" checklist (["on"] when enabled).
        current_figure: Current figure state from live-graph.
        last_goal_xy_km: Previously stored goal as [x, y] or None on first run.
        stored_range: Previously stored range as {"x": [xmin, xmax], "y": [ymin, ymax]}
                      or None on first run
        reached_global_keys: Previously stored keys of reached global waypoints, or None on
                      first run.

    Returns:
        (fig, new_goal_as_list, last_range, reached_global_keys, path_status):
            - fig: The updated Plotly figure
            - new_goal_as_list: [x, y] for storage in dcc.Store (JSON serializable)
            - last_range: [x-range, y-range] for storage in dcc.Store (JSON serializable)
            - reached_global_keys: list of reached global waypoint keys for storage in dcc.Store
            - path_status: remaining waypoint count and latest replan reason

    """
    global queue, _latest_vs  # noqa

    # Check which input triggered the callback
    ctx = dash.callback_context
    triggered_id = ctx.triggered[0]["prop_id"].split(".")[0] if ctx.triggered else None

    # Interval update (default behavior)
    if queue is not None and not queue.empty():
        vs = queue.get()  # type: ignore
        _latest_vs = vs
    elif triggered_id in ("map-toggle", "reset-button") and _latest_vs is not None:
        # A view-only control changed but no new ROS message arrived; re-render the last frame
        # so the toggle/reset takes effect immediately instead of waiting for the next message.
        vs = _latest_vs
    else:
        return dash.no_update, dash.no_update, stored_range, dash.no_update, dash.no_update

    last_goal_tuple = (
        cs.XY(last_goal_xy_km[0], last_goal_xy_km[1]) if last_goal_xy_km is not None else None
    )

    show_map = map_toggle is not None and "on" in map_toggle

    if relayout_data and triggered_id == "live-graph":
        # Only process relayout_data if it was the actual trigger
        required_keys = [
            "xaxis.range[0]",
            "xaxis.range[1]",
            "yaxis.range[0]",
            "yaxis.range[1]",
        ]
        if all(key in relayout_data for key in required_keys):
            last_range = {
                "x": [
                    relayout_data["xaxis.range[0]"],
                    relayout_data["xaxis.range[1]"],
                ],
                "y": [
                    relayout_data["yaxis.range[0]"],
                    relayout_data["yaxis.range[1]"],
                ],
            }
        else:
            last_range = stored_range
    else:
        last_range = stored_range

    fig, new_goal_xy, reached_global_keys = build_figure(
        vs, last_goal_tuple, last_range, reached_global_keys, show_map
    )

    if triggered_id == "reset-button":
        if current_figure is None:
            return (
                dash.no_update,
                dash.no_update,
                dash.no_update,
                dash.no_update,
                dash.no_update,
            )

        min_bounds, max_bounds = get_state_space_bounds(vs)
        x_range = [min_bounds.x, max_bounds.x]
        y_range = [min_bounds.y, max_bounds.y]
        last_range = {
            "x": x_range,
            "y": y_range,
        }
        fig.update_layout(
            xaxis=dict(range=x_range, autorange=False),
            yaxis=dict(range=y_range, autorange=False),
            uirevision="reset",
        )

    remaining_waypoints = vs.latest_msg.remaining_waypoints
    replan_reason = vs.last_replan_reason or "None"
    path_status = (
        f"Remaining Waypoints: {remaining_waypoints}\n Last Replan Reason: {replan_reason}"
    )
    return fig, [new_goal_xy[0], new_goal_xy[1]], last_range, reached_global_keys, path_status


@app.callback(
    Output("wind-status", "children"),
    Input("apply-wind-btn", "n_clicks"),
    State("tw-dir-input", "value"),
    State("tw-speed-input", "value"),
    prevent_initial_call=True,
)
def update_wind(_, tw_dir_deg, tw_speed_kmph):
    try:
        if tw_dir_deg is None or tw_speed_kmph is None:
            return "Invalid input"

        if not (-180 < tw_dir_deg <= 180):
            return "Direction must be (-180, 180]°"

        if tw_speed_kmph < 0:
            return "Speed must be ≥ 0"

        write_wind_params(tw_dir_deg, tw_speed_kmph)

        return f"Applied wind: {tw_dir_deg}°, {tw_speed_kmph} km/h"

    except Exception as e:
        return f"Error: {e}"


app.clientside_callback(
    """
    function(status_text) {
        if (!status_text) return "";

        // Wait 5000ms (5 seconds) then find the div and wipe it
        setTimeout(function(){
            const statusDiv = document.getElementById('wind-status');
            if (statusDiv) statusDiv.innerText = "";
        }, 5000);

        return status_text;
    }
    """,
    Output("wind-status", "children", allow_duplicate=True),
    Input("wind-status", "children"),
    prevent_initial_call=True,
)


@app.callback(
    Output("gps-status", "children"),
    Input("apply-gps-btn", "n_clicks"),
    State("gps-toggles", "value"),
    State("drift-speed-input", "value"),
    State("drift-dir-input", "value"),
    State("drift-accel-input", "value"),
    prevent_initial_call=True,
)
def update_gps_params(_, toggles, drift_speed, drift_dir, drift_accel):
    try:
        if drift_speed is None or drift_dir is None or drift_accel is None:
            return "Fill in all numeric fields"
        if drift_speed < 0:
            return "Drift speed must be ≥ 0"
        if not (-180 < drift_dir <= 180):
            return "Drift direction must be in (-180, 180]°"

        active = set(toggles or [])
        use_gps_noise = "use_gps_noise" in active
        use_ocean_drift = "use_ocean_drift" in active
        use_drift_randomization = "use_drift_randomization" in active

        apply_gps_params(
            use_gps_noise,
            use_ocean_drift,
            use_drift_randomization,
            drift_speed,
            drift_dir,
            drift_accel,
        )

        parts = [
            f"Noise: {'on' if use_gps_noise else 'off'}",
            f"Drift: {'on' if use_ocean_drift else 'off'}",
        ]
        if use_ocean_drift:
            parts.append(f"Rand: {'on' if use_drift_randomization else 'off'}")
            parts.append(f"{drift_speed} km/h @ {drift_dir}° {drift_accel} km/h²")
        return "✓ " + " | ".join(parts)

    except Exception as e:
        return f"Error: {e}"


app.clientside_callback(
    """
    function(status_text) {
        if (!status_text) return "";
        setTimeout(function(){
            const statusDiv = document.getElementById('gps-status');
            if (statusDiv) statusDiv.innerText = "";
        }, 5000);
        return status_text;
    }
    """,
    Output("gps-status", "children", allow_duplicate=True),
    Input("gps-status", "children"),
    prevent_initial_call=True,
)
