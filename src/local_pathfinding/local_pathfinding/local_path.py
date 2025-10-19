"""The path to the next global waypoint, represented by the LocalPath class."""

from typing import List, Optional

import custom_interfaces.msg as ci
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import MultiPolygon, LineString
from pyproj import Geod

import local_pathfinding.obstacles as ob
from local_pathfinding.ompl_path import OMPLPath
import local_pathfinding.coord_systems as cs
import math

LOW_WIND_SPEED_THRESHOLD = 9.26  # 5 knots
GEODESIC = Geod(ellps="WGS84")
LOCAL_WAYPOINT_REACHED_THRESH_KM = 0.5


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

        if not (global_path and global_path.waypoints):
            raise ValueError("Cannot create a LocalPathState with an empty global_path")
        self.global_path = global_path
        self.reference_latlon = target_global_waypoint

        if not planner:
            raise ValueError("planner must not be None")
        self.planner = planner

        # obstacles are initialized by OMPLPath right before solving
        self.obstacles: List[ob.Obstacle] = []


class LocalPath:
    """Sets and updates the OMPL path and the local waypoints

    Attributes:
        _logger (RcutilsLogger): ROS logger.
        _ompl_path (Optional[OMPLPath]): Raw representation of the path from OMPL.
        _waypoint_index
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
    def calculate_desired_heading_and_waypoint_index(
        path: ci.Path, waypoint_index: int, boat_lat_lon: ci.HelperLatLon
    ):
        waypoint = path.waypoints[waypoint_index]
        desired_heading, _, distance_to_waypoint_m = GEODESIC.inv(
            boat_lat_lon.longitude, boat_lat_lon.latitude, waypoint.longitude, waypoint.latitude
        )

        if cs.meters_to_km(distance_to_waypoint_m) < LOCAL_WAYPOINT_REACHED_THRESH_KM:
            # If we reached the current local waypoint, aim for the next one
            waypoint_index += 1
            waypoint = path.waypoints[waypoint_index]
            desired_heading, _, distance_to_waypoint_m = GEODESIC.inv(
                boat_lat_lon.longitude,
                boat_lat_lon.latitude,
                waypoint.longitude,
                waypoint.latitude,
            )

        return cs.bound_to_180(desired_heading), waypoint_index

    @staticmethod
    def in_collision_zone(local_wp_index, reference_latlon, path, obstacles):
        """
        Checks if the stored path is in a collision zone or not

        Returns:
            boolean: True if the path intersects a collision zone
        """
        # print(f"path: {path}")
        path = list(map(lambda x: (cs.latlon_to_xy(reference_latlon, x)), path.waypoints))
        for i in range(local_wp_index, len(path) - 1):
            p1, p2 = path[i], path[i + 1]
            p1, p2 = (p1.x, p1.y), (p2.x, p2.y)
            segment = LineString([p1, p2])

            # print(f"reference_latlon: {reference_latlon}")
            # print(f"local_wp_index: {local_wp_index}")
            for o in obstacles:
                # print(f"obstacles: {o.print_info()}")
                if segment.crosses(o.collision_zone) or segment.touches(o.collision_zone):
                    return True

        return False

    def update_if_needed(
        self,
        gps: ci.GPS,
        ais_ships: ci.AISShips,
        global_path: ci.Path,
        local_waypoint_index: int,
        received_new_global_waypoint: bool,
        target_global_waypoint: ci.HelperLatLon,
        filtered_wind_sensor: ci.WindSensor,
        planner: str,
        land_multi_polygon: MultiPolygon = None,
    ) -> tuple[Optional[float], Optional[int]] :
        """Updates the OMPL path, waypoints and current state. The path is updated if a new path
            is found. Returns true if the path is updated and false otherwise.

        Args:
            gps (ci.GPS): GPS data.
            ais_ships (ci.AISShips): AIS ships data.
            global_path (ci.Path): Path to the destination.
            filtered_wind_sensor (ci.WindSensor): Wind data.
        """
        # this raises ValueError if any of the parameters are not properly initialized
        self._waypoint_index = local_waypoint_index
        state = LocalPathState(
            gps, ais_ships, global_path, target_global_waypoint, filtered_wind_sensor, planner
        )
        self.state = state
        ompl_path = OMPLPath(
            parent_logger=self._logger,
            max_runtime=1.0,
            local_path_state=state,
            land_multi_polygon=land_multi_polygon,
        )
        old_ompl_path = self._ompl_path

        heading_new_path, wp_index = self.calculate_desired_heading_and_waypoint_index(
            ompl_path.get_path(), 0, gps.lat_lon
        )

        if received_new_global_waypoint:
            print("new global waypoint")
            self._logger.debug("Updating local path")
            self._update(ompl_path)
            return heading_new_path, wp_index

        if old_ompl_path is None:
            # continue on the same path
            print("old path is none")
            self._update(ompl_path)
            return heading_new_path, wp_index

        heading_old_path, updated_wp_index = self.calculate_desired_heading_and_waypoint_index(
            old_ompl_path.get_path(), local_waypoint_index, gps.lat_lon
        )
        # check if the current path goes through a collision zone.
        # No need to check for new path since it's fresh and ompl doesn't generate path that
        # go through a collision zone
        if self.in_collision_zone(local_waypoint_index, self.state.reference_latlon, self.path, self.state.obstacles):
            print("old path is in collision zone")
            self._update(ompl_path)
            return heading_new_path, wp_index

        # if self.state.wind_speed < LOW_WIND_SPEED_THRESHOLD:
        #     print("windspeed is low, no need to change the path")
        #     return heading_old_path, updated_wp_index

        heading_diff_old_path = cs.calculate_heading_dff(self.state.heading, heading_old_path)
        heading_diff_new_path = cs.calculate_heading_dff(self.state.heading, heading_new_path)

        old_cost = old_ompl_path.get_cost(updated_wp_index)
        new_cost = ompl_path.get_cost(wp_index)

        max_cost = max(old_cost, new_cost, 1)
        old_cost_normalized = old_cost/max_cost
        new_cost_normalized = new_cost/max_cost

        max_heading_diff = max(math.fabs(heading_diff_new_path),
                               math.fabs(heading_diff_old_path),
                               1.0)

        heading_diff_new_normalized = heading_diff_new_path/max_heading_diff
        heading_diff_old_normalized = heading_diff_old_path/max_heading_diff

        w_h = 0.6
        w_c = 0.4

        metric_old = w_h * heading_diff_old_normalized + w_c * old_cost_normalized
        metric_new = w_h * heading_diff_new_normalized + w_c * new_cost_normalized

        if metric_new < metric_old:
            print(
                f"New path is cheaper, updating local path "
                f"(old cost: {old_cost:.2f}, "
                f"new cost: {new_cost:.2f})"
                f", metric_old: {metric_old:.2f}, "
                f"metric_new: {metric_new:.2f}, "
                f"old_cost_normalized: {old_cost_normalized:.2f}, "
                f"new_cost_normalized: {new_cost_normalized:.2f}"
            )
            self._update(ompl_path)
            return heading_new_path, wp_index
        else:
            print(
                f"old path is cheaper, continuing on the same path"
                f"(old cost: {old_cost:.2f}, "
                f"new cost: {new_cost:.2f})"
                f", metric_old: {metric_old:.2f}, "
                f"metric_new: {metric_new:.2f}, "
                f"old_cost_normalized: {old_cost_normalized:.2f}, "
                f"new_cost_normalized: {new_cost_normalized:.2f}"
            )
            return heading_old_path, wp_index

    def _update(self, ompl_path: OMPLPath):
        print("switching path")
        self._ompl_path = ompl_path
        self.path = self._ompl_path.get_path()
