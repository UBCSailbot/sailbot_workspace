"""The path to the next global waypoint, represented by the LocalPath class."""

from typing import List, Optional

import custom_interfaces.msg as ci
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.obstacles as ob
from local_pathfinding.ompl_path import OMPLPath


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
        if ompl_path.solved:
            self._logger.debug("Updating local path")
            self._update(ompl_path)
            return True
        return False

    def _update(self, ompl_path: OMPLPath):
        self._ompl_path = ompl_path
        self.path = self._ompl_path.get_path()
