"""The path to the next global waypoint, represented by the LocalPath class."""

from typing import List, Optional

import custom_interfaces.msg as ci
from pyproj import Geod
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
from local_pathfinding.ompl_path import OMPLPath

GEODESIC = Geod(ellps="WGS84")
HEADING_DIFFERENCE_THRESH_DEGREES = 0.1
REDUCTION_FACTOR_THRESH = 0.9


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
        self.reference_latlon = self.global_path.waypoints[-1]

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
        path (Path): Collection of coordinates that form the local path to the next
                          global waypoint.
        state (LocalPathState): the current local path state.
    """

    def __init__(self, parent_logger: RcutilsLogger):
        self._logger = parent_logger.get_child(name="local_path")
        self._ompl_path: Optional[OMPLPath] = None
        self.path: Optional[ci.Path] = None
        self.state: Optional[LocalPathState] = None

    def update_if_needed(
        self,
        gps: ci.GPS,
        ais_ships: ci.AISShips,
        global_path: ci.Path,
        filtered_wind_sensor: ci.WindSensor,
        planner: str,
    ) -> bool:
        """Updates the OMPL path, waypoints and current state. The path is updated if a new path
            is found. Returns true if the path is updated and false otherwise.

        Args:
            gps (ci.GPS): GPS data.
            ais_ships (ci.AISShips): AIS ships data.
            global_path (ci.Path): Path to the destination.
            filtered_wind_sensor (ci.WindSensor): Wind data.
        """
        # this raises ValueError if any of the parameters are not properly initialized
        state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor, planner)
        self.state = state
        ompl_path = OMPLPath(
            parent_logger=self._logger,
            max_runtime=1.0,
            local_path_state=state,
        )

        if (self._ompl_path is None) or (not self.old_path_is_valid(gps, ais_ships, global_path)):
            self._update(ompl_path)
            return True

        old_path_cost, new_path_cost = self._ompl_path.get_cost(), ompl_path.get_cost()
        new_path_1st_waypoint = ompl_path.get_path().waypoints[0]  # First waypoint is 0 right?

        if self.new_path_changes_desired_heading(gps.lat_lon, new_path_1st_waypoint):
            path_cost_reduction = 1 - (old_path_cost - new_path_cost) / old_path_cost
            if path_cost_reduction > REDUCTION_FACTOR_THRESH:
                self._update(ompl_path)
                return True
        elif new_path_cost < old_path_cost:
            self._update
            return True

        self._logger.info("Continuing on old local path")
        return False

    def old_path_is_valid(self, gps: ci.GPS, ais_ships: ci.AISShips, global_path: ci.Path) -> bool:
        """Checks if the old path is valid. A path is valid if:
        - It does not pass through any boat collision zones. Note: Land won't move into path.
        - Sailbot has not drifted away from the path
        - The path reaches the next global waypoint"""

        if self.ais_ship_crosses_old_path(gps, ais_ships):
            return False

        if self.sailbot_far_from_old_path(gps):
            return False

        if self.new_global_waypoint():
            return False

        return True

    def ais_ship_crosses_old_path(self, gps: ci.GPS, ais_ships: ci.AISShips):
        return False

    def sailbot_far_from_old_path(self, gps):
        return False

    def new_global_waypoint(self):
        return False

    def new_path_changes_desired_heading(
        self, boat: ci.HelperLatLon, new_path_1st_waypoint: ci.HelperLatLon
    ) -> bool:

        if self._ompl_path is None:
            return True

        old_path_first_waypoint = self._ompl_path.get_path().waypoints[0]

        old_desired_heading, _, _ = GEODESIC.inv(
            boat.longitude,
            boat.latitude,
            old_path_first_waypoint.longitude,
            old_path_first_waypoint.latitude,
        )
        new_desired_heading, _, _ = GEODESIC.inv(
            boat.longitude,
            boat.latitude,
            new_path_1st_waypoint.longitude,
            new_path_1st_waypoint.latitude,
        )
        return new_desired_heading - old_desired_heading > HEADING_DIFFERENCE_THRESH_DEGREES

    def min_dist_from_path_km(self, gps: ci.GPS) -> float:
        """Checks each line segment in the local path, computes"""

        if self._ompl_path is None:
            raise AttributeError

        waypoints = self._ompl_path.get_path().waypoints
        for wp_num in range(0, len(waypoints) - 1):
            wp1, wp2 = cs.latlon_to_xy(gps._lat_lon, waypoints[wp_num]), cs.latlon_to_xy(
                gps._lat_lon, waypoints[wp_num + 1]
            )
            # See my onenote

        return 0.0

    def _update(self, ompl_path: OMPLPath):
        self._ompl_path = ompl_path
        self.path = self._ompl_path.get_path()
        self._logger.info("Updating local path")
