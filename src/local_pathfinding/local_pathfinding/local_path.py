"""The path to the next global waypoint, represented by the `LocalPath` class."""

from typing import List, Optional, Tuple

from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

from local_pathfinding.ompl_path import OMPLPath


class LocalPathState:
    """Gathers and stores the state of Sailbot.
    The attributes' units and conventions can be found in the ROS msgs they are derived from in the
    custom_interfaces repository.

    Attributes:
        `position` (Tuple[float, float]): Latitude and longitude of Sailbot.
        `speed` (float): Speed of Sailbot.
        `heading` (float): Direction that Sailbot is pointing.
        `ais_ships` (List[HelperAISShip]): Information about nearby ships.
        `global_path` (List[Tuple[float, float]]): Path to the destination that Sailbot is
            navigating along.
        `wind_speed` (float): Wind speed.
        `wind_direction` (int): Wind direction.
        `planner` (str): Planner to use for the OMPL query.
    """

    def __init__(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: Path,
        filtered_wind_sensor: WindSensor,
        planner: str,
    ):
        """Initializes the state from ROS msgs."""
        if gps:  # TODO: remove when mock can be run
            self.position = (gps.lat_lon.latitude, gps.lat_lon.longitude)
            self.speed = gps.speed.speed
            self.heading = gps.heading.heading

        if ais_ships:  # TODO: remove when mock can be run
            self.ais_ships = [ship for ship in ais_ships.ships]

        if global_path:  # TODO: remove when mock can be run
            self.global_path = [
                HelperLatLon(latitude=waypoint.latitude, longitude=waypoint.longitude)
                for waypoint in global_path.waypoints
            ]
        else:
            self.global_path = global_path

        if filtered_wind_sensor:  # TODO: remove when mock can be run
            self.wind_speed = filtered_wind_sensor.speed.speed
            self.wind_direction = filtered_wind_sensor.direction

        self.planner = planner


class LocalPath:
    """Sets and updates the OMPL path and the local waypoints

    Attributes:
        `_logger` (RcutilsLogger): ROS logger.
        `_ompl_path` (Optional[OMPLPath]): Raw representation of the path from OMPL.
        `waypoints` (Optional[List[Tuple[float, float]]]): List of coordinates that form the path
            to the next global waypoint.
    """

    def __init__(self, parent_logger: RcutilsLogger):
        """Initializes the LocalPath class."""
        self._logger = parent_logger.get_child(name="local_path")
        self._ompl_path: Optional[OMPLPath] = None
        self.waypoints: Optional[List[Tuple[float, float]]] = None

    def update_if_needed(
        self,
        gps: GPS,
        ais_ships: AISShips,
        global_path: Path,
        filtered_wind_sensor: WindSensor,
        planner: str,
    ):
        """Updates the OMPL path and waypoints. The path is updated if a new path is found.

        Args:
            `gps` (GPS): GPS data.
            `ais_ships` (AISShips): AIS ships data.
            `global_path` (Path): Path to the destination.
            `filtered_wind_sensor` (WindSensor): Wind data.
        """
        state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor, planner)
        ompl_path = OMPLPath(
            parent_logger=self._logger,
            max_runtime=1.0,
            local_path_state=state,
        )
        if ompl_path.solved:
            self._logger.info("Updating local path")
            self._update(ompl_path)
            return True

        return False

    def _update(self, ompl_path: OMPLPath):
        self._ompl_path = ompl_path
        self.waypoints = self._ompl_path.get_waypoints()
