"""The path to the next global waypoint, represented by the `LocalPath` class."""

import logging
from typing import List, Optional, Tuple

from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import LineString, Point

import local_pathfinding.coord_systems as cs
from local_pathfinding.obstacles import Boat
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
        # TODO: Remove these to make logger right
        if parent_logger and hasattr(parent_logger, "get_child"):
            self._logger = parent_logger.get_child(name="local_path")
        else:
            # Use the logger as-is or create a default logger if None is provided
            self._logger = parent_logger or logging.getLogger("local_path")
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
        if self.check_wind_valid(filtered_wind_sensor=filtered_wind_sensor):
            if not self.old_path_valid(gps, ais_ships, global_path):
                ompl_path = self._solve(gps, ais_ships, global_path, filtered_wind_sensor, planner)
                if ompl_path.solved:
                    self._logger.info("Updating local path")
                    self._update(ompl_path)
            else:
                self._logger.info("Continuing on old local path")

    def _solve(self, gps, ais_ships, global_path, filtered_wind_sensor, planner):
        state = LocalPathState(gps, ais_ships, global_path, filtered_wind_sensor, planner)
        ompl_path = OMPLPath(
            parent_logger=self._logger,
            max_runtime=1.0,
            local_path_state=state,
        )
        return ompl_path

    def _update(self, ompl_path: OMPLPath):
        self._ompl_path = ompl_path
        self.waypoints = self._ompl_path.get_waypoints()

    def check_wind_valid(self, filtered_wind_sensor: WindSensor) -> bool:
        """Checks if the wind speed is too low for sailing.

        Args:
            `filtered_wind_sensor` (WindSensor): Filtered wind sensor data.

        Returns:
            bool: False if wind speed is too low, True otherwise.
        """
        if filtered_wind_sensor is None:
            self._logger.warning("Filtered wind sensor is None. Not computing OMPL Path")
            return False

        if filtered_wind_sensor.speed.speed < 5:
            # TODO Example threshold of 5.0 for low speed. Change accordingly
            self._logger.warning(
                f"Wind speed too low: {filtered_wind_sensor.speed.speed}. Not computing OMPL Path"
            )
            return False
        return True

    def old_path_valid(self, gps: GPS, ais_ships: AISShips, global_path: Path) -> bool:
        """Checks if the old path is still valid based on multiple conditions.

        Args:
            `gps` (GPS): GPS data.
            `ais_ships` (AISShips): AIS ships data.
            `global_path` (Path): Path to the destination.
            `filtered_wind_sensor` (WindSensor): Wind data.

        Returns:
            bool: False if old path is invalid, True if old path is still valid.
        """
        if self.waypoints is None:
            return False
        else:
            # TODO Buffer is set to 2.0 km. Change accordingly
            if (
                self.old_path_in_collision_zone(gps, ais_ships)
                or self.sailbot_drifted_from_old_path(gps, self.waypoints, buffer=2.0)
                or self.global_path_changed(global_path, self.waypoints, buffer=2.0)
            ):
                return False
            return True

    def old_path_in_collision_zone(self, gps: GPS, ais_ships: AISShips) -> bool:
        """
        Checks if the old path is in a collision zone based on the provided AISShips data.

        This method creates a collision zone for each AIS ship using the Boat class and checks if
        the SailBot's old path, represented as a series of waypoints, intersects or is within any
        of the collision zones

        Args:
            gps (GPS): GPS message containing the SailBot's current position, speed, and heading.
            ais_ships (AISShips): AISShips message containing a list of detected ships.

        Returns:
            bool: True if the old path is in a collision zone False otherwise.
        """
        self.sailbot_position_latlon = gps.lat_lon
        self.sailbot_speed = gps.speed.speed
        self.reference = gps.lat_lon

        reference_latlon = HelperLatLon(
            latitude=gps.lat_lon.latitude, longitude=gps.lat_lon.longitude
        )

        if len(self.waypoints) < 2:  # type: ignore
            single_waypoint = self.waypoints[0]  # type: ignore
            projected_point = cs.latlon_to_xy(
                reference_latlon,
                HelperLatLon(latitude=single_waypoint[0], longitude=single_waypoint[1]),
            )
            old_path_geom = Point(projected_point)
        else:
            projected_waypoints = [
                cs.latlon_to_xy(reference_latlon, HelperLatLon(latitude=lat, longitude=lon))
                for lat, lon in self.waypoints  # type: ignore
            ]
            old_path_geom = LineString(projected_waypoints)

        for ais_ship in ais_ships.ships:
            boat = Boat(
                reference=reference_latlon,
                sailbot_position=gps.lat_lon,
                sailbot_speed=gps.speed.speed,
                ais_ship=ais_ship,
            )
            boat.update_boat_collision_zone()

            if old_path_geom.intersects(boat.collision_zone) or old_path_geom.within(
                boat.collision_zone
            ):
                self._logger.debug(
                    f"Old path is in collision zone with AIS ship ID: {ais_ship.id}"
                )
                return True

        self._logger.debug("Old path is not in collision zone with any AIS ships.")
        return False

    def global_path_changed(
        self,
        global_path: Path,
        waypoints: List[Tuple[float, float]],
        buffer: float = 2.0,
    ) -> bool:
        """Checks if the global path has changed since the last update.

        Args:
            `global_path` (Path): Global path to the destination.
            'waypoints' : Old path waypoints
            `buffer` (float): Buffer in km (default is 2.0 km).

        Returns:
            bool: True if the global path has changed, False otherwise.
        """
        # Get the last waypoint from self.waypoints
        last_local_waypoint = HelperLatLon(latitude=waypoints[-1][0], longitude=waypoints[-1][1])

        # Check if the last local waypoint is within the buffer of any global path waypoint
        for global_waypoint in global_path.waypoints:
            global_latlon = HelperLatLon(
                latitude=global_waypoint.latitude, longitude=global_waypoint.longitude
            )

            _, _, distance_m = cs.GEODESIC.inv(
                last_local_waypoint.longitude,
                last_local_waypoint.latitude,
                global_latlon.longitude,
                global_latlon.latitude,
            )
            if distance_m <= buffer * 1000:  # km coverted to m
                return False

        self._logger.warning(
            f"None of the waypoints in global path within {buffer} km with local goal state"
        )
        return True

    def sailbot_drifted_from_old_path(
        self, gps: GPS, waypoints: List[Tuple[float, float]], buffer: float = 2.0
    ) -> bool:
        """Checks if the Sailbot has drifted buffer distance away from the old path.

        Args:
            'gps' (GPS): GPS data.
            'waypoints' : Old path waypoints
            'buffer' (float): Distance from the path in 'km'

        Returns:
            bool: True if Sailbot has drifted significantly, False otherwise.
        """

        reference_latlon = HelperLatLon(latitude=waypoints[0][0], longitude=waypoints[0][1])

        projected_waypoints = [
            cs.latlon_to_xy(reference_latlon, HelperLatLon(latitude=lat, longitude=lon))
            for lat, lon in waypoints
        ]

        path_line = LineString([(pt.x, pt.y) for pt in projected_waypoints])

        path_polygon = path_line.buffer(buffer)

        projected_gps = cs.latlon_to_xy(
            reference_latlon,
            HelperLatLon(latitude=gps.lat_lon.latitude, longitude=gps.lat_lon.longitude),
        )

        gps_point = Point(projected_gps.x, projected_gps.y)

        # Check if the Sailbot is outside the buffered path
        if not path_polygon.contains(gps_point):
            self._logger.warning(f"Sailbot has drifted from the old path: GPS {gps.lat_lon}")
            return True

        self._logger.debug("Sailbot is within the path polygon.")
        return False
