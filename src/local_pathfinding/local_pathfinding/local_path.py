"""The path to the next global waypoint, represented by the LocalPath class."""

import math
from collections import deque
from datetime import datetime, timedelta
from typing import List, Optional

from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import LineString, MultiPolygon

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
from local_pathfinding.wind_coord_systems import Wind
from local_pathfinding.ompl_path import OMPLPath

WIND_SPEED_CHANGE_THRESH_PROP = 0.3
WIND_DIRECTION_CHANGE_THRESH_DEG = 10
WIND_HISTORY_LEN = 30
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
        aw_history (List[Wind]): History of wind sensor readings
            (Queue with max length WIND_HISTORY_LEN).
        aw_avg (Optional[Wind]): Average of the wind history, used for path planning.
            Speed is in kmph, direction in degrees. Is None until aw_history reaches full capacity
            (WIND_HISTORY_LEN readings). Once full, updated every time a new wind sensor reading is
            added to aw_history.
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
        self.aw_history: deque = deque(maxlen=WIND_HISTORY_LEN)
        self.aw_avg: Optional[Wind] = None
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

        new_wind_data = Wind(self.wind_speed, self.wind_direction)
        self.update_aw_history(new_wind_data)

    def update_aw_history(self, current_wind: Wind):
        """Updates apparent wind history and recalculates the average wind. The wind values are
        all apparent wind.

        Maintains a history of up to WIND_HISTORY_LEN wind readings. When the history
        exceeds the max length, the oldest reading is removed.

        Args:
            current_wind (Wind): Current wind speed (kmph) and direction (deg)
        """

        self.aw_history.append(current_wind)

        # Recalculate average wind from history once minimum wind readings reached
        self.aw_avg = self._calculate_aw_avg()

    def _calculate_aw_avg(self) -> Optional[Wind]:
        """Calculates the average apparent wind from the wind history once the deque is full.

        Returns:
            Optional[Wind]: Average wind object, or None if aw_history is not full
        """
        if len(self.aw_history) < WIND_HISTORY_LEN:
            return None

        avg_speed, sin_sum, cos_sum = 0.0, 0.0, 0.0

        for wind in self.aw_history:
            avg_speed += wind.speed_kmph / len(self.aw_history)
            # Use circular mean to handle wrap-around
            sin_sum += math.sin(math.radians(wind.dir_deg))
            cos_sum += math.cos(math.radians(wind.dir_deg))

        avg_direction = math.degrees(math.atan2(sin_sum, cos_sum))
        avg_direction = cs.bound_to_180(avg_direction)

        return Wind(avg_speed, avg_direction)


class LocalPath:
    """Sets and updates the OMPL path and the local waypoints

    Attributes:
        _logger (RcutilsLogger): ROS logger.
        _ompl_path (Optional[OMPLPath]): Raw representation of the path from OMPL.
        _target_lp_wp_index (int): 0-based array index of the local waypoint Polaris is
            currently heading toward. This is initialized to 1 because OMPL path index 0 is the
            start state near the boat, and index 1 is the first target waypoint.
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
        path: ci.Path, target_lp_wp_index: int, boat_lat_lon: ci.HelperLatLon
    ):
        """Calculates the desired heading using GEODESIC. Updates the waypoint index (i.e. change
        the waypoint) if the boat is close enough to the current target waypoint.

        Args:
            path (ci.Path): Array of waypoints
            target_lp_wp_index (int): 0-based array index of the local waypoint Polaris is
                currently heading toward. This should start at index 1 because index 0 is the
                OMPL start waypoint near the boat.
            boat_lat_lon (ci.HelperLatLon): boat coordinates

        Returns:
            tuple[float, int]: Desired heading in degrees and the updated local
                waypoint index (target_lp_wp_index).
        Raises:
            ValueError: if the path is None
            IndexError: If index out of bounds or path is None
        """
        if path is None:
            raise ValueError("Path is None")
        if target_lp_wp_index < 1 or target_lp_wp_index >= len(path.waypoints):
            raise IndexError("target_lp_wp_index must be in [1, len(path.waypoints) - 1]")

        waypoint = path.waypoints[target_lp_wp_index]
        desired_heading, _, distance_to_waypoint_m = cs.GEODESIC.inv(
            boat_lat_lon.longitude, boat_lat_lon.latitude, waypoint.longitude, waypoint.latitude
        )

        if cs.meters_to_km(distance_to_waypoint_m) < LOCAL_WAYPOINT_REACHED_THRESH_KM:
            # If we reached the target local waypoint, update to the next target waypoint
            target_lp_wp_index += 1

            if target_lp_wp_index >= len(path.waypoints):
                raise IndexError("waypoint idx > len(path.waypoints). Must generate new path")

            waypoint = path.waypoints[target_lp_wp_index]
            desired_heading, _, distance_to_waypoint_m = cs.GEODESIC.inv(
                boat_lat_lon.longitude,
                boat_lat_lon.latitude,
                waypoint.longitude,
                waypoint.latitude,
            )

        return cs.bound_to_180(desired_heading), target_lp_wp_index

    def in_collision_zone(self):
        """
        Checks if the path intersects a collision zone.

        Uses the current `self.path`, `self.state.obstacles`, and
        `self._target_lp_wp_index` to test path segments that have not yet been traversed.

        Returns:
            bool: True if the path intersects a collision zone, False otherwise.
        """
        xy_path = [cs.latlon_to_xy(self.state.reference_latlon, wp) for wp in self.path.waypoints]
        segment_start_index = max(self._target_lp_wp_index - 1, 0)
        for i in range(segment_start_index, len(xy_path) - 1):
            p1, p2 = xy_path[i], xy_path[i + 1]
            segment = LineString([(p1.x, p1.y), (p2.x, p2.y)])
            for o in self.state.obstacles:
                if segment.crosses(o.collision_zone) or segment.touches(o.collision_zone):
                    self._logger.debug("Path intersects with collision zone")
                    return True
        return False

    @staticmethod
    def is_significant_wind_change(
        new_tw_data: Wind,
        previous_tw_data: Wind,
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
            new_tw_data (Wind): Current wind speed/direction that may require a change in path
            previous_tw_data (Wind): Wind speed/direction when the current path was generated

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

    def must_change_path(self, received_new_global_waypoint: bool) -> tuple[bool, str]:
        """Check if the path must be changed.

        Returns:
            tuple[bool, str]: (should_change_path, reason_description)
        """
        if received_new_global_waypoint:
            return True, "Received new global waypoint"
        if self._ompl_path is None:
            return True, "OMPL path is None"
        if self.state is None:
            return True, "State is None"
        if self.path is None:
            return True, "Path is None"
        if self.in_collision_zone():
            return True, "Path intersects collision zone"
        if self.is_path_expired():
            return True, "Path has expired (TTL exceeded)"
        if self._target_lp_wp_index < 1:
            return True, f"Target waypoint index too low: {self._target_lp_wp_index}"
        if self._target_lp_wp_index >= len(self.path.waypoints):
            return (
                True,
                f"Target waypoint index out of bounds: {self._target_lp_wp_index} >= {len(self.path.waypoints)}", # noqa
            )

        return False, "Path is valid, no change needed"

    def update_if_needed(
        self,
        gps: ci.GPS,
        ais_ships: ci.AISShips,
        global_path: ci.Path,
        target_lp_wp_index: int,
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
            target_lp_wp_index (int): 0-based array index of the local waypoint Polaris is
                currently heading toward. This starts at index 1 because OMPL index 0 is the
                start state near the boat.
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
        self._target_lp_wp_index = target_lp_wp_index
        old_ompl_path = self._ompl_path

        new_state = LocalPathState(
            gps, ais_ships, global_path, target_global_waypoint, filtered_wind_sensor, planner
        )
        new_ompl_path = OMPLPath(
            parent_logger=self._logger,
            local_path_state=new_state,
            land_multi_polygon=land_multi_polygon,
        )
        # TODO: handle the error thrown here in reworked update_if_needed
        try:
            heading_new_path, new_target_lp_wp_index = self.calculate_desired_heading_and_wp_index(
                new_ompl_path.get_path(), 1, gps.lat_lon
            )
        except (ValueError, IndexError):
            # TODO: handle this after merging Jai's PR that sets sail to False
            heading_new_path, new_target_lp_wp_index = None, -1

        must_change, reason = self.must_change_path(received_new_global_waypoint)
        if must_change:
            self._logger.debug(f"Updating local path: {reason}")
            self.state = new_state
            self._update(new_ompl_path)
            return heading_new_path, new_target_lp_wp_index
        else:
            self.state.update_state(gps, ais_ships, filtered_wind_sensor)  # type: ignore

        try:
            heading_old_path, old_target_lp_wp_index = self.calculate_desired_heading_and_wp_index(
                self._ompl_path.get_path(), target_lp_wp_index, gps.lat_lon  # type: ignore
            )
        except (ValueError, IndexError):
            return heading_new_path, new_target_lp_wp_index

        heading_diff_old_path = cs.calculate_heading_diff(
            self.state.heading,  # type: ignore
            heading_old_path,
        )
        heading_diff_new_path = cs.calculate_heading_diff(
            self.state.heading,  # type: ignore
            heading_new_path,
        )

        old_cost = old_ompl_path.get_remaining_cost(  # type: ignore
            old_target_lp_wp_index, gps.lat_lon
        )
        new_cost = new_ompl_path.get_remaining_cost(new_target_lp_wp_index, gps.lat_lon)
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
            return heading_new_path, new_target_lp_wp_index
        else:
            self._logger.debug("old path is cheaper, continuing on the same path")
            return heading_old_path, old_target_lp_wp_index

    def _update(self, ompl_path: OMPLPath):

        self._ompl_path = ompl_path
        self.path = self._ompl_path.get_path()
