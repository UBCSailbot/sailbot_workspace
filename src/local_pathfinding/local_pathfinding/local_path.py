"""The path to the next global waypoint, represented by the LocalPath class."""

import math
from datetime import datetime, timedelta
from typing import List, Optional

from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import LineString, MultiPolygon

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
import local_pathfinding.wind_coord_systems as wcs
from local_pathfinding.ompl_path import OMPLPath

WIND_SPEED_CHANGE_THRESH_PROP = 0.3
WIND_DIRECTION_CHANGE_THRESH_DEG = 10
LOCAL_WAYPOINT_REACHED_THRESH_KM = 0.5
HEADING_WEIGHT = 0.6
COST_WEIGHT = 0.4
PATH_TTL_SEC = timedelta(seconds=600)


class LocalPathState:
    """Stores the current state of Sailbot's navigation data.
    The attributes' units and conventions can be found in the ROS msgs they are derived from in the
    custom_interfaces package.

    Attributes:
        position (ci.HelperLatLon): Latitude and longitude of Sailbot.
        speed (float): Speed of Sailbot.
        heading (float): Direction that Sailbot is pointing.
        ais_ships (List[HelperAISShip]): Information about nearby ships.
        global_path (List[Tuple[float, float]]): Path to the destination that Sailbot is
                                                 navigating along.
        target_global_waypoint (ci.HelperLatLon): The global waypoint that we are heading towards.
            The global waypoint is the same as the reference latlon.
        wind_speed (float): Wind speed.
        wind_direction (int): Wind direction.
        planner (str): Planner to use for the OMPL query.
        reference (ci.HelperLatLon): Lat and lon position of the next global waypoint.
        obstacles (List[Obstacle]): All obstacles in the state space
        path_generated_time (datetime): Time when the path was generated
    """

    def __init__(
        self,
        gps: ci.GPS,
        ais_ships: ci.AISShips,
        global_path: ci.Path,
        target_global_waypoint: ci.HelperLatLon,
        filtered_wind_sensor: ci.WindSensor,
        planner: str,
    ):
        self.update_state(gps, ais_ships, filtered_wind_sensor)

        if not (global_path and global_path.waypoints):
            raise ValueError("Cannot create a LocalPathState with an empty global_path")
        self.global_path = global_path
        self.reference_latlon = target_global_waypoint

        if not planner:
            raise ValueError("planner must not be None")
        self.planner = planner

        # obstacles are initialized by OMPLPath right before solving
        self.obstacles: List[ob.Obstacle] = []
        self.path_generated_time = datetime.now()

    def update_state(
        self, gps: ci.GPS, ais_ships: ci.AISShips, filtered_wind_sensor: ci.WindSensor
    ):
        """Updates the changeable environment without changing the path or reference_latlon

        This method updates only the dynamic state variables (position, heading, speed,
        ais_ships, wind) without changing the reference coordinate system or global path.
        Used when continuing on an existing path to avoid coordinate system mismatches.

        Args:
            gps (ci.GPS): Current GPS position and heading data
            ais_ships (ci.AISShips): Updated AIS ship data
            filtered_wind_sensor (ci.WindSensor): Updated wind sensor data
        """
        if not gps:
            raise ValueError("gps must not be None")
        self.position = gps.lat_lon
        self.speed = gps.speed.speed
        self.heading = gps.heading.heading

        if not ais_ships:
            raise ValueError("ais_ships must not be None")
        self.ais_ships = [ship for ship in ais_ships.ships]

        if not filtered_wind_sensor:
            raise ValueError("filtered_wind_sensor must not be None")
        self.wind_speed = filtered_wind_sensor.speed.speed
        self.wind_direction = filtered_wind_sensor.direction


class LocalPath:
    """Sets and updates the OMPL path and the local waypoints

    Attributes:
        _logger (RcutilsLogger): ROS logger.
        _ompl_path (Optional[OMPLPath]): Raw representation of the path from OMPL.
        _prev_lp_wp_index (int): Index of the local waypoint that Polaris has already traversed.
            Polaris heads toward the waypoint at index `_prev_lp_wp_index + 1`.
        path (Path): Collection of coordinates that form the local path to the next
                          global waypoint.
        state (LocalPathState): the current local path state.
    """

    def __init__(self, parent_logger: RcutilsLogger):
        self._logger = parent_logger.get_child(name="local_path")
        self._ompl_path: Optional[OMPLPath] = None
        self.path: Optional[ci.Path] = None
        self.state: Optional[LocalPathState] = None

    def is_path_expired(self) -> bool:
        """Check if the current path has exceeded the PATH_TTL timeout.

        Returns:
            bool: True if the path has expired, False otherwise.
        """
        if self.state is None:
            self._logger.debug("Path is expired, since the state is None")
            return True
        is_expired = datetime.now() >= (self.state.path_generated_time + PATH_TTL_SEC)
        if is_expired:
            self._logger.debug("Path is expired")
        return is_expired

    @staticmethod
    def calculate_desired_heading_and_wp_index(
        path: ci.Path, prev_lp_wp_index: int, boat_lat_lon: ci.HelperLatLon
    ):
        """Calculates the desired heading using GEODESIC. Updates the waypoint index (i.e. change
        the waypoint) if the boat is close enough to the next waypoint.
        The caller MUST ensure that prev_lp_wp_index <= len(path.waypoints)

        Args:
            path (ci.Path): Array of waypoints
            prev_lp_wp_index (int): index of the local waypoint that Polaris has already traversed
            in the path array
            (i.e. the waypoint sailbot traversed, sailbot is heading towards waypoint_index + 1)
            boat_lat_lon (ci.HelperLatLon): boat coordinates

        Returns:
            tuple[float, int]: Desired heading in degrees and the updated local
                waypoint index (prev_lp_wp_index).
        Raises:
            IndexError: If index out of bounds or path is None
        """
        if path is None:
            raise ValueError("Path is None")

        waypoint = path.waypoints[prev_lp_wp_index]
        desired_heading, _, distance_to_waypoint_m = cs.GEODESIC.inv(
            boat_lat_lon.longitude, boat_lat_lon.latitude, waypoint.longitude, waypoint.latitude
        )

        if cs.meters_to_km(distance_to_waypoint_m) < LOCAL_WAYPOINT_REACHED_THRESH_KM:
            # If we reached the next local waypoint, update prev_lp_wp_index to the next waypoint
            prev_lp_wp_index += 1

            if prev_lp_wp_index > len(path.waypoints):
                raise IndexError("waypoint idx > len(path.waypoints). Must generate new path")

            waypoint = path.waypoints[prev_lp_wp_index]
            desired_heading, _, distance_to_waypoint_m = cs.GEODESIC.inv(
                boat_lat_lon.longitude,
                boat_lat_lon.latitude,
                waypoint.longitude,
                waypoint.latitude,
            )

        return cs.bound_to_180(desired_heading), prev_lp_wp_index

    def in_collision_zone(self):
        """
        Checks if the path intersects a collision zone.

        Uses the current `self.path`, `self.state.obstacles`, and
        `self._prev_lp_wp_index` to test path segments that have not yet been traversed.

        Returns:
            bool: True if the path intersects a collision zone, False otherwise.
        """
        self._logger.debug("Path in collision zone")
        xy_path = list(
            map(
                lambda lat_lon: (cs.latlon_to_xy(self.reference_latlon, lat_lon)),
                self.path.waypoints,
            )
        )
        for i in range(self._prev_lp_wp_index, len(xy_path) - 1):
            p1, p2 = xy_path[i], xy_path[i + 1]
            segment = LineString([(p1.x, p1.y), (p2.x, p2.y)])
            for o in self.state.obstacles:
                if segment.crosses(o.collision_zone) or segment.touches(o.collision_zone):
                    return True
        return False

    @staticmethod
    def is_significant_wind_change(
        new_tw_data: wcs.Wind,
        previous_tw_data: wcs.Wind,
    ) -> bool:
        """Returns true if there is a significant change in the true wind warranting a change in
           path. Although this function works with any kind of wind, it is specifically designed
           for true wind data.

        Evaluates new wind speed compared to the wind condition used when previous
        path was made.

        The criteria to determine if the wind change is significant include:
        - A change in wind speed exceeding WIND_SPEED_CHANGE_THRESH_PROP of the previous speed
          used to calculate previous path
        - A change in wind direction exceeding WIND_DIRECTION_CHANGE_THRESH_DEG degrees

        Args:
            new_tw_data (wcs.Wind): Current wind speed/direction that may require a change in path
            previous_tw_data (wcs.Wind): Wind speed/direction when the current path was generated

        Returns:
            boolean: True if there is a significant change in the wind, False otherwise.
        """

        # Check for significant changes
        prev_tw_speed_kmph = previous_tw_data.speed_kmph
        prev_tw_dir_deg = previous_tw_data.dir_deg

        current_tw_speed_kmph = new_tw_data.speed_kmph
        current_tw_dir_deg = new_tw_data.dir_deg

        speed_change_ratio = abs(prev_tw_speed_kmph - current_tw_speed_kmph) / prev_tw_speed_kmph
        dir_change = abs(cs.bound_to_180(current_tw_dir_deg - prev_tw_dir_deg))

        return (
            speed_change_ratio >= WIND_SPEED_CHANGE_THRESH_PROP
            or dir_change >= WIND_DIRECTION_CHANGE_THRESH_DEG
        )

    def must_change_path(self):
        return (
            (self._ompl_path is None)
            or (self.state is None)
            or (self.path is None)
            or (self.in_collision_zone())
            or (self.is_path_expired())
            or (self._prev_lp_wp_index > len(self.path.waypoints))
        )

    def update_if_needed(
        self,
        gps: ci.GPS,
        ais_ships: ci.AISShips,
        global_path: ci.Path,
        prev_lp_wp_index: int,
        received_new_global_waypoint: bool,
        target_global_waypoint: ci.HelperLatLon,
        filtered_wind_sensor: ci.WindSensor,
        planner: str,
        land_multi_polygon: MultiPolygon = None,
    ) -> tuple[Optional[float], Optional[int]]:
        """Updates the local path using OMPL if conditions warrant a path change.

        Evaluates whether to update the current path based on several criteria:
        - Receipt of a new global waypoint
        - Absence of an existing path
        - Current path intersecting with collision zones
        - New path having a better cost-heading metric than the old path

        The decision metric combines normalized heading difference and
        normalized path cost to balance directional efficiency with
        obstacle avoidance. The weights can be changed to tune the system better.

            gps (ci.GPS): Current GPS position and heading data.
            ais_ships (ci.AISShips): AIS data for nearby ships (obstacles).
            global_path (ci.Path): The global path plan to the destination.
            prev_lp_wp_index (int): Index of the last traversed local waypoint.
            The boat is heading towards the waypoint at index `prev_lp_wp_index + 1`.
            received_new_global_waypoint (bool): Flag indicating if a new global
                waypoint was received.
            target_global_waypoint (ci.HelperLatLon): Target waypoint from global path.
            filtered_wind_sensor (ci.WindSensor): Filtered wind speed and direction data.
            planner (str): Name of the OMPL planner to use.
            land_multi_polygon (MultiPolygon, optional): Polygon representing land masses
                to avoid. Defaults to None.

        Returns:
            tuple[Optional[float], Optional[int]]: A tuple containing:
                - Desired heading in degrees
                - Updated waypoint index
            The method decides whether to return the heading for new path or old path
        """
        self._prev_lp_wp_index = prev_lp_wp_index
        old_ompl_path = self._ompl_path

        new_state = LocalPathState(
            gps, ais_ships, global_path, target_global_waypoint, filtered_wind_sensor, planner
        )
        new_ompl_path = OMPLPath(
            parent_logger=self._logger,
            local_path_state=new_state,
            land_multi_polygon=land_multi_polygon,
        )
        heading_new_path, new_prev_lp_wp_index = self.calculate_desired_heading_and_wp_index(
            new_ompl_path.get_path(), 0, gps.lat_lon
        )
        if self.must_change_path() or received_new_global_waypoint:
            if received_new_global_waypoint:
                self._logger.debug("Updating local path because we have a new global waypoint")
            else:
                self._logger.debug("old path is None")
            self.state = new_state
            self._update(new_ompl_path)
            return heading_new_path, new_prev_lp_wp_index
        else:
            # this assert will never fail!! Added this here to make Pylance happy
            assert self.state is not None
            self.state.update_state(gps, ais_ships, filtered_wind_sensor)

        try:
            # Same as the previous assert, Python can't statically analyze the fact that
            # self._ompl_path won't be None for some reason. Removing this assert will lead to
            # pylance/LSP red squiggles
            assert self._ompl_path is not None
            heading_old_path, old_prev_lp_wp_index = self.calculate_desired_heading_and_wp_index(
                self._ompl_path.get_path(), prev_lp_wp_index, gps.lat_lon
            )
        except ValueError:
            return heading_new_path, new_prev_lp_wp_index

        if self.is_path_expired():
            self._logger.debug("Updating local path because PATH_TTL has expired")
            self._update(new_ompl_path)
            return heading_new_path, new_prev_lp_wp_index

        assert old_ompl_path is not None

        heading_diff_old_path = cs.calculate_heading_diff(self.state.heading, heading_old_path)
        heading_diff_new_path = cs.calculate_heading_diff(self.state.heading, heading_new_path)

        old_cost = old_ompl_path.get_remaining_cost(old_prev_lp_wp_index, gps.lat_lon)
        new_cost = new_ompl_path.get_remaining_cost(new_prev_lp_wp_index, gps.lat_lon)
        max_cost = max(old_cost, new_cost, 1)
        old_cost_normalized = old_cost / max_cost
        new_cost_normalized = new_cost / max_cost

        max_heading_diff = max(
            math.fabs(heading_diff_new_path), math.fabs(heading_diff_old_path), 1.0
        )

        heading_diff_new_normalized = heading_diff_new_path / max_heading_diff
        heading_diff_old_normalized = heading_diff_old_path / max_heading_diff

        w_h = HEADING_WEIGHT
        w_c = COST_WEIGHT

        metric_old = w_h * heading_diff_old_normalized + w_c * old_cost_normalized
        metric_new = w_h * heading_diff_new_normalized + w_c * new_cost_normalized

        self._logger.debug(
            f"(old cost: {old_cost:.2f}, "
            f"new cost: {new_cost:.2f})"
            f", metric_old: {metric_old:.2f}, "
            f"metric_new: {metric_new:.2f}, "
            f"old_cost_normalized: {old_cost_normalized:.2f}, "
            f"new_cost_normalized: {new_cost_normalized:.2f}"
        )
        if metric_new < metric_old:
            self._logger.debug("New path is cheaper, updating local path ")
            self.state = new_state
            self._update(new_ompl_path)
            return heading_new_path, new_prev_lp_wp_index
        else:
            self._logger.debug("old path is cheaper, continuing on the same path")
            return heading_old_path, old_prev_lp_wp_index

    def _update(self, ompl_path: OMPLPath):

        self._ompl_path = ompl_path
        self.path = self._ompl_path.get_path()
