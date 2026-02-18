"""The path to the next global waypoint, represented by the LocalPath class."""

import math
from typing import List, Optional

import custom_interfaces.msg as ci
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import LineString, MultiPolygon

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
from local_pathfinding.ompl_path import OMPLPath

LOCAL_WAYPOINT_REACHED_THRESH_KM = 0.5
HEADING_WEIGHT = 0.6
COST_WEIGHT = 0.4


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
        _last_lp_wp_index: Local waypoint index (i.e. pointer to the next local waypoint that the
        boat is following)
        path (Path): Collection of coordinates that form the local path to the next
                          global waypoint.
        state (LocalPathState): the current local path state.
    """

    def __init__(self, parent_logger: RcutilsLogger):
        self._logger = parent_logger.get_child(name="local_path")
        self._ompl_path: Optional[OMPLPath] = None
        self.path: Optional[ci.Path] = None
        self.state: Optional[LocalPathState] = None

    @staticmethod
    def calculate_desired_heading_and_wp_index(
        path: ci.Path, waypoint_index: int, boat_lat_lon: ci.HelperLatLon
    ):
        """Calculates the desired heading using GEODESIC. Updates the waypoint index (i.e. change
        the waypoint) if the boat is close enough to the current waypoint.

        Args:
            path (ci.Path): Array of waypoints
            waypoint_index (int): Pointer to the current local waypoint index in path array
            (i.e. the waypoint sailbot traversed, sailbot is heading towards waypoint_index + 1)
            boat_lat_lon (ci.HelperLatLon): boat coordinates

        Returns:
            _type_: _description_
        """
        waypoint = path.waypoints[waypoint_index]
        desired_heading, _, distance_to_waypoint_m = cs.GEODESIC.inv(
            boat_lat_lon.longitude, boat_lat_lon.latitude, waypoint.longitude, waypoint.latitude
        )

        if cs.meters_to_km(distance_to_waypoint_m) < LOCAL_WAYPOINT_REACHED_THRESH_KM:
            # If we reached the current local waypoint, aim for the next one
            waypoint_index += 1
            waypoint = path.waypoints[waypoint_index]
            desired_heading, _, distance_to_waypoint_m = cs.GEODESIC.inv(
                boat_lat_lon.longitude,
                boat_lat_lon.latitude,
                waypoint.longitude,
                waypoint.latitude,
            )

        return cs.bound_to_180(desired_heading), waypoint_index

    @staticmethod
    def in_collision_zone(
        local_wp_index: int,
        reference_latlon: ci.HelperLatLon,
        path: ci.Path,
        obstacles: List[ob.Obstacle],
    ):
        """
        Checks if the path is in a collision zone or not.

        Args:
            local_wp_index (int): Index of the current local waypoint in the path.
            reference_latlon (ci.HelperLatLon): lat lon of the next global waypoint
            path (ci.Path): Collection of waypoints forming the local path.
            obstacles (List[Obstacle]): List of obstacles in the state space.

        Returns:
            boolean: True if the path intersects a collision zone, False otherwise.
        """
        xy_path = list(
            map(lambda lat_lon: (cs.latlon_to_xy(reference_latlon, lat_lon)), path.waypoints)
        )
        for i in range(local_wp_index, len(xy_path) - 1):
            p1, p2 = xy_path[i], xy_path[i + 1]
            segment = LineString([(p1.x, p1.y), (p2.x, p2.y)])
            for o in obstacles:
                if segment.crosses(o.collision_zone) or segment.touches(o.collision_zone):
                    return True
        return False

    def update_if_needed(
        self,
        gps: ci.GPS,
        ais_ships: ci.AISShips,
        global_path: ci.Path,
        last_lp_wp_index: int,
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
            last_lp_wp_index (int): Current index in the local waypoint list.
            This is the index that the boat last traversed. The boat is heading towards
            the index following last_lp_wp_index
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
        self._last_lp_wp_index = last_lp_wp_index
        old_ompl_path = self._ompl_path

        # If we need to generate a new path or don't have an existing state
        if (
            (received_new_global_waypoint or old_ompl_path is None)
            or (self.path is None)
            or (self.state is None)
        ):
            # Create a new state with the new target_global_waypoint
            new_state = LocalPathState(
                gps, ais_ships, global_path, target_global_waypoint, filtered_wind_sensor, planner
            )
            new_ompl_path = OMPLPath(
                parent_logger=self._logger,
                local_path_state=new_state,
                land_multi_polygon=land_multi_polygon,
            )
            heading_new_path, wp_index = self.calculate_desired_heading_and_wp_index(
                new_ompl_path.get_path(), 0, gps.lat_lon
            )
            if received_new_global_waypoint:
                self._logger.debug("Updating local path because we have a new global waypoint")
            else:
                self._logger.debug("old path is none")
            self.state = new_state
            self._update(new_ompl_path)
            return heading_new_path, wp_index
        else:
            self.state.update_state(gps, ais_ships, filtered_wind_sensor)

        # Create a new state for evaluating the new path candidate
        new_state = LocalPathState(
            gps, ais_ships, global_path, target_global_waypoint, filtered_wind_sensor, planner
        )
        new_ompl_path = OMPLPath(
            parent_logger=self._logger,
            local_path_state=new_state,
            land_multi_polygon=land_multi_polygon,
        )

        heading_new_path, wp_index = self.calculate_desired_heading_and_wp_index(
            new_ompl_path.get_path(), 0, gps.lat_lon
        )
        heading_old_path, updated_wp_index = self.calculate_desired_heading_and_wp_index(
            old_ompl_path.get_path(), last_lp_wp_index, gps.lat_lon
        )

        # Check if the current path goes through a collision zone
        # Use the old path's reference for collision checking
        if self.in_collision_zone(
            last_lp_wp_index, self.state.reference_latlon, self.path, self.state.obstacles
        ):
            self._logger.debug("old path is in collision zone")
            self.state = new_state
            self._update(new_ompl_path)
            return heading_new_path, wp_index

        heading_diff_old_path = cs.calculate_heading_diff(self.state.heading, heading_old_path)
        heading_diff_new_path = cs.calculate_heading_diff(self.state.heading, heading_new_path)

        old_last_wp_index = max(updated_wp_index - 1, 0)
        new_last_wp_index = max(wp_index - 1, 0)

        old_cost = old_ompl_path.get_remaining_cost(old_last_wp_index, gps.lat_lon)
        new_cost = new_ompl_path.get_remaining_cost(new_last_wp_index, gps.lat_lon)
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
            return heading_new_path, wp_index
        else:
            self._logger.debug("old path is cheaper, continuing on the same path")
            # Keep the old state with old reference (already updated with update_state)
            return heading_old_path, updated_wp_index

    def _update(self, ompl_path: OMPLPath):

        self._ompl_path = ompl_path
        self.path = self._ompl_path.get_path()
