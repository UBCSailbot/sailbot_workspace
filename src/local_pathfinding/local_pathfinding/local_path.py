"""The path to the next global waypoint, represented by the LocalPath class."""

import math
from collections import deque
from dataclasses import dataclass
from time import monotonic
from typing import Callable, List, Optional

import numpy as np
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import get_logger
from shapely.geometry import LineString, MultiPolygon

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
from local_pathfinding.ompl_path import OMPLPath
from local_pathfinding.wind_coord_systems import (
    Wind,
    aw_gc_to_tw_gc,
    boat_to_global_coordinate,
)

WIND_SPEED_CHANGE_THRESH_PROP = 0.3
WIND_SPEED_CHANGE_THRESH_OFFSET_KMPH = 2.0
WIND_DIRECTION_CHANGE_THRESH_DEG = 10
WIND_HISTORY_LEN = 30
SEGMENT_DEVIATION_THRESHOLD = 0.3
GPS_POSITION_ERROR_KM = 0.003
LOCAL_WAYPOINT_REACHED_THRESH_KM = 0.05
HEADING_WEIGHT = 0.6
COST_WEIGHT = 0.4
PATH_TTL_SEC = 600.0
MAX_OMPL_PATH_GEN_TRIES = 2


class PathNotFoundError(Exception):
    """Raised when a usable local path is unavailable."""


@dataclass
class MustChangeReason:
    should_change_path: bool
    reason: str


class WindTracker:
    """Tracks true wind readings and exposes a rolling average.

    The tracker owns wind history separately from LocalPathState so the history
    survives LocalPathState replacement during path regeneration. Until the
    history reaches WIND_HISTORY_LEN readings, tw_avg remains None.

    The caller is responsible for converting wind to the true wind before passing it
        to this tracker.

    Attributes:
        tw_history (deque[Wind]): History of wind sensor readings converted to true wind
            with max length WIND_HISTORY_LEN.
        tw_avg (Optional[Wind]): Average of tw_history, used for path planning.
            Speed is in kmph, direction is in degrees. This is None until tw_history reaches
            full capacity. Once full, it is updated every time a new true wind reading is
            added to tw_history.
        using_one_tw_point (bool): Whether the current accepted path was generated before the
            rolling average was populated, using only one true wind point.
    """

    def __init__(self) -> None:
        self.tw_history: deque = deque(maxlen=WIND_HISTORY_LEN)
        self.tw_avg: Optional[Wind] = None
        self.using_one_tw_point: bool = True
        self._logger = get_logger("wind_tracker")

    def update_tw_history(self, new_tw: Wind):
        """Updates wind history and recalculates the average wind.

        Maintains a history of up to WIND_HISTORY_LEN true wind readings. When the history
        exceeds the max length, the oldest reading is removed.

        Args:
            new_tw (Wind): Newest true-wind reading with speed (kmph) and a
                global flow-toward direction (deg). Caller is responsible for
                ensuring this is true wind.
        """

        self.tw_history.append(new_tw)

        self.tw_avg = self._calculate_wind_avg()

        self._logger.debug(
            f"WindTracker updated: new_tw speed={new_tw.speed_kmph:.2f} km/h, "
            f"direction={new_tw.dir_deg:.2f} deg, "
            f"history_len={len(self.tw_history)}/{WIND_HISTORY_LEN}, "
            f"tw_avg={self.tw_avg}"
        )

    def _calculate_wind_avg(self) -> Optional[Wind]:
        """Calculates the average wind from the wind history once the deque is full.

        Returns:
            Optional[Wind]: Average wind object, or None if wind_history is not full
        """
        if len(self.tw_history) < WIND_HISTORY_LEN:
            return None

        avg_speed, sin_sum, cos_sum = 0.0, 0.0, 0.0

        for wind in self.tw_history:
            avg_speed += wind.speed_kmph / len(self.tw_history)
            # Use circular mean to handle wrap-around (https://en.wikipedia.org/wiki/Circular_mean)
            sin_sum += math.sin(math.radians(wind.dir_deg))
            cos_sum += math.cos(math.radians(wind.dir_deg))

        avg_direction = math.degrees(math.atan2(sin_sum, cos_sum))
        avg_direction = cs.bound_to_180(avg_direction)

        return Wind(avg_speed, avg_direction)


@dataclass
class LocalPathInputs:
    """Current navigation inputs needed to update or regenerate a local path.

    Attributes:
        gps (ci.GPS): Current GPS position and speed data.
        heading (ci.HelperHeading): Current e-compass boat heading from the ``rudder`` topic.
        ais_ships (Optional[ci.AISShips]): AIS data for nearby ships, if available.
        global_path (Optional[ci.Path]): The global path plan to the destination, if available.
        target_global_waypoint (Optional[ci.HelperLatLon]): Target waypoint from the global path,
            if available.
        filtered_wind_sensor (Optional[ci.WindSensor]): Filtered apparent wind data, if available.
        land_multi_polygon (Optional[MultiPolygon]): Optional land masses to avoid.
    """

    gps: ci.GPS
    heading: ci.HelperHeading
    ais_ships: ci.AISShips | None
    global_path: ci.Path | None
    target_global_waypoint: ci.HelperLatLon | None
    filtered_wind_sensor: ci.WindSensor | None
    land_multi_polygon: Optional[MultiPolygon] = None


class LocalPathState:
    """Stores the current state of Sailbot's navigation data.
    The attributes' units and conventions can be found in the ROS msgs they are derived from in the
    custom_interfaces package.

    Attributes:
        position (ci.HelperLatLon): Latitude and longitude of Sailbot.
        speed (float): Speed of Sailbot.
        heading (float): Direction that Sailbot is pointing.
        ais_ships (List[HelperAISShip]): Information about nearby ships.
        global_path (ci.Path): Path to the destination that Sailbot is navigating along.
        reference_latlon (ci.HelperLatLon): The global waypoint that Sailbot is heading toward.
            The global waypoint is the same as the reference latlon.
        obstacles (List[Obstacle]): All obstacles in the state space.
        path_generated_time_sec (float): Clock time in seconds when the path was generated.
        current_tw (Wind): Latest true-wind reading converted from the filtered
            apparent-wind sensor. Its global direction points where air travels.
        wind_tracker (WindTracker): Rolling true wind tracker shared across path states.
            It owns tw_history, a queue of wind readings with max length WIND_HISTORY_LEN, and
            tw_avg, the rolling wind average used for path planning once tw_history
            reaches full capacity.
            It also tracks whether the current accepted path was generated from one cold-start
            true wind point or from the rolling average.
        path_generated_wind (Wind): Flow-toward true-wind value used to generate the current path.
            This is the rolling average when available; otherwise it falls back to current_tw.
    """

    def __init__(
        self,
        gps: ci.GPS,
        heading: ci.HelperHeading,
        ais_ships: ci.AISShips,
        global_path: ci.Path,
        target_global_waypoint: ci.HelperLatLon,
        filtered_wind_sensor: ci.WindSensor,
        wind_tracker: WindTracker,
        path_generated_time_sec: Optional[float] = None,
    ):
        self.wind_tracker = wind_tracker
        self.update_state(gps, heading, ais_ships, filtered_wind_sensor)
        self.path_generated_wind = self.wind_tracker.tw_avg or self.current_tw

        if not (global_path and global_path.waypoints):
            raise ValueError("Cannot create a LocalPathState with an empty global_path")
        self.global_path = global_path
        self.reference_latlon = target_global_waypoint

        # obstacles are initialized by OMPLPath right before solving
        self.obstacles: List[ob.Obstacle] = []
        self.path_generated_time_sec = (
            monotonic() if path_generated_time_sec is None else path_generated_time_sec
        )
        self._logger = get_logger("local_path_state")

    def update_state(
        self,
        gps: ci.GPS,
        heading: ci.HelperHeading,
        ais_ships: ci.AISShips,
        filtered_wind_sensor: ci.WindSensor,
    ) -> None:
        """Updates the changeable environment without changing the path or reference_latlon

        This method updates only the dynamic state variables (position, heading, speed,
        ais_ships, wind) without changing the reference coordinate system or global path.
        Wind history is updated by LocalPath before this method is called so LocalPathState
        construction and OMPL retries do not duplicate wind readings.

        Args:
            gps (ci.GPS): Current GPS position and speed data.
            heading (ci.HelperHeading): Current e-compass boat heading from ``rudder``.
            ais_ships (ci.AISShips): Updated AIS ship data
            filtered_wind_sensor (ci.WindSensor): Updated wind sensor data
        """
        if not gps:
            raise ValueError("gps must not be None")
        if not heading:
            raise ValueError("heading must not be None")
        self.position = gps.lat_lon
        self.speed = gps.speed.speed
        self.heading = heading.heading

        if not ais_ships:
            raise ValueError("ais_ships must not be None")
        self.ais_ships = [ship for ship in ais_ships.ships]

        if not filtered_wind_sensor:
            raise ValueError("filtered_wind_sensor must not be None")
        current_aw = Wind(filtered_wind_sensor.speed.speed, filtered_wind_sensor.direction)
        current_aw_dir_deg_gc = boat_to_global_coordinate(self.heading, current_aw.dir_deg)
        tw_dir_deg, tw_speed_kmph = aw_gc_to_tw_gc(
            aw_dir_deg_gc=current_aw_dir_deg_gc,
            aw_speed_kmph=current_aw.speed_kmph,
            boat_heading_deg_gc=self.heading,
            boat_speed_kmph=self.speed,
        )
        self.current_tw = Wind(tw_speed_kmph, tw_dir_deg)

    def update_obstacles(self) -> None:
        """Refresh obstacles from the latest AIS data while reusing land geometry."""
        if not self.obstacles:
            self._logger.warn(
                "update_obstacles called with no existing obstacles: skipping refresh. "
                "This should only happen before the first successful replan."
            )
            return

        self.obstacles = ob.update_boat_obstacles(
            obstacles=self.obstacles,
            reference=self.reference_latlon,
            sailbot_position=self.position,
            sailbot_speed=self.speed,
            ais_ships=self.ais_ships,
        )


class LocalPath:
    """Sets and updates the OMPL path and the local waypoints

    Attributes:
        _logger (RcutilsLogger): ROS logger.
        _now_sec (Callable[[], float]): Returns an increasing time in seconds, used only for
            elapsed-time differences (path age / TTL and the switch-duration log). In the running
            node this is the ROS system clock (seconds since the Unix epoch); it falls back to
            time.monotonic (arbitrary reference) when no clock is injected, e.g. in tests.
        _ompl_path (Optional[OMPLPath]): Raw representation of the path from OMPL.
        _target_lp_wp_index (int): 0-based array index of the local waypoint Polaris is
            currently heading toward. This is set by update_if_needed. It usually starts at 1
            because OMPL path index 0 is the start state near the boat, and index 1 is the first
            target waypoint.
        path (Optional[ci.Path]): Collection of coordinates that form the local path to the next
                          global waypoint.
        state (Optional[LocalPathState]): the current local path state.
    """

    def __init__(
        self,
        parent_logger: RcutilsLogger,
        now_sec: Optional[Callable[[], float]] = None,
    ):
        self._logger = parent_logger.get_child(name="local_path")
        self._now_sec = now_sec or monotonic
        self._ompl_path: Optional[OMPLPath] = None
        self.path: Optional[ci.Path] = None
        self.state: Optional[LocalPathState] = None
        self.last_replan_reason: str = ""
        self.last_remaining_waypoints: int = 0

    def _count_remaining_waypoints(self) -> int:
        """Returns waypoints remaining on the current path from the current target index."""
        if self.path is None or not self.path.waypoints:
            return 0
        return max(len(self.path.waypoints) - max(self._target_lp_wp_index, 0), 0)

    def is_path_expired(self) -> bool:
        """Check if the current path has exceeded the PATH_TTL timeout.

        Returns:
            bool: True if the path has expired, False otherwise.
        """
        if self.state is None:
            self._logger.info("Path is expired, since the state is None")
            return True
        is_expired = self._now_sec() >= (self.state.path_generated_time_sec + PATH_TTL_SEC)
        if is_expired:
            self._logger.info("Path is expired")
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
            PathNotFoundError: If `path` is None or `target_lp_wp_index` is outside the
                valid range `[1, len(path.waypoints) - 1]`.
            IndexError: If the current waypoint is reached and incrementing
                `target_lp_wp_index` requires generating a new path.
        """
        if path is None:
            raise PathNotFoundError("Path is None")
        if target_lp_wp_index < 1 or target_lp_wp_index >= len(path.waypoints):
            raise PathNotFoundError("target_lp_wp_index must be in [1, len(path.waypoints) - 1]")

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
                    self._logger.info("Path intersects with collision zone")
                    return True
        return False

    def is_significant_wind_change(
        self,
        new_wind_data: Wind,
        previous_wind_data: Wind,
    ) -> bool:
        """Returns true if a wind change is significant enough to warrant a path change.

        This helper compares two wind readings without converting between wind types or coordinate
        frames. It works for true wind, apparent wind, or any other wind representation as long as
        both readings use the same representation.

        The criteria to determine if the wind change is significant include:
        - A change in wind speed exceeding WIND_SPEED_CHANGE_THRESH_PROP of the previous speed
          used to calculate previous path plus WIND_SPEED_CHANGE_THRESH_OFFSET_KMPH
        - A change in wind direction exceeding WIND_DIRECTION_CHANGE_THRESH_DEG degrees

        Args:
            new_wind_data (Wind): Current wind speed/direction that may require a path change.
            previous_wind_data (Wind): Wind speed/direction used for the current path. Must be the
                same wind type and coordinate frame as new_wind_data.

        Returns:
            boolean: True if there is a significant change in the wind, False otherwise.
        """
        # Check for significant changes
        previous_speed_kmph = previous_wind_data.speed_kmph
        previous_dir_deg = previous_wind_data.dir_deg

        current_speed_kmph = new_wind_data.speed_kmph
        current_dir_deg = new_wind_data.dir_deg

        speed_change = abs(previous_speed_kmph - current_speed_kmph)
        speed_change_threshold = (
            WIND_SPEED_CHANGE_THRESH_PROP * previous_speed_kmph
            + WIND_SPEED_CHANGE_THRESH_OFFSET_KMPH
        )
        dir_change = abs(cs.bound_to_180(current_dir_deg - previous_dir_deg))

        significant_change = (
            speed_change >= speed_change_threshold
            or dir_change >= WIND_DIRECTION_CHANGE_THRESH_DEG
        )

        if significant_change:
            self._logger.info(
                "Significant wind change detected: "
                f"speed {previous_wind_data.speed_kmph:.2f} -> {current_speed_kmph:.2f} km/h "
                f"({speed_change:.2f} km/h, threshold {speed_change_threshold:.2f} km/h); "
                f"direction {previous_dir_deg:.2f} -> {current_dir_deg:.2f} deg "
                f"({dir_change:.2f} deg, threshold {WIND_DIRECTION_CHANGE_THRESH_DEG:.2f} deg)"
            )
        else:
            self._logger.debug(
                f"speed {previous_wind_data.speed_kmph:.2f} -> {current_speed_kmph:.2f} km/h "
                f"({speed_change:.2f} km/h, threshold {speed_change_threshold:.2f} km/h); "
                f"direction {previous_dir_deg:.2f} -> {current_dir_deg:.2f} deg "
                f"({dir_change:.2f} deg, threshold {WIND_DIRECTION_CHANGE_THRESH_DEG:.2f} deg)"
            )

        return significant_change

    def exceeded_segment_deviation(
        self, path: ci.Path, target_lp_wp_index: int, boat_lat_lon: ci.HelperLatLon
    ) -> bool:
        """Returns true if the boat has deviated from the path segment by more than
        SEGMENT_DEVIATION_THRESHOLD * length of segment in kilometers.

        This function calculates the shortest distance from the boat's current position to the
        line segment defined by start_xy and end_xy. If this distance exceeds the defined
        threshold, it indicates that the boat has deviated significantly from the intended path
        segment.

        Args:
            path (ci.Path): Array of waypoints
            target_lp_wp_index (int): 0-based array index of the local waypoint Polaris is
                currently heading toward. This should start at index 1 because index 0 is the
                OMPL start waypoint near the boat.
            boat_lat_lon (ci.HelperLatLon): boat coordinates

        Raises:
            IndexError: If target_lp_wp_index is 0 or >= len(path.waypoints), since a
            valid preceding/target waypoint is required to define the segment.
        """
        if target_lp_wp_index == 0 or target_lp_wp_index >= len(path.waypoints):
            self._logger.warn(
                "Target waypoint out of bounds, must be in range [1, len(waypoints))"
            )  # noqa
            raise IndexError("Target waypoint out of bounds, must be in range [1, len(waypoints))")

        prev_wp = path.waypoints[target_lp_wp_index - 1]
        target_wp = path.waypoints[target_lp_wp_index]

        _, _, segment_length_m = cs.GEODESIC.inv(
            prev_wp.longitude,
            prev_wp.latitude,
            target_wp.longitude,
            target_wp.latitude,
        )
        segment_length_km = cs.meters_to_km(segment_length_m)
        max_deviation_km = segment_length_km * SEGMENT_DEVIATION_THRESHOLD + GPS_POSITION_ERROR_KM

        # Build a local XY frame with prev_wp as origin.
        target_xy_km = cs.latlon_to_xy(prev_wp, target_wp)
        boat_xy_km = cs.latlon_to_xy(prev_wp, boat_lat_lon)

        target_wp_vector = np.array([target_xy_km.x, target_xy_km.y])
        boat_vector = np.array([boat_xy_km.x, boat_xy_km.y])

        segment_len_sq = np.linalg.norm(target_wp_vector) ** 2

        if segment_len_sq == 0.0:
            return False  # Waypoints are the same, no deviation possible

        # Project the boat position onto the segment vector, clamped to [0, 1] so the
        # closest point stays within the segment bounds rather than extending past either end.
        projection_factor = np.dot(boat_vector, target_wp_vector) / segment_len_sq
        projection_factor = max(0.0, min(1.0, projection_factor))

        # The closest point on the segment to the boat (prev_wp is origin, no offset needed)
        projection_vector = projection_factor * target_wp_vector
        rejection_vector = boat_vector - projection_vector
        distance_to_segment_km = np.linalg.norm(rejection_vector)

        segment_exceeded = distance_to_segment_km > max_deviation_km
        if segment_exceeded:
            self._logger.info(
                "Boat deviated from path segment: "
                f"distance {distance_to_segment_km:.2f} km, "
                f"max deviation {max_deviation_km:.2f} km"
            )

        return segment_exceeded

    def must_change_path(
        self,
        received_new_global_waypoint: bool,
        boat_lat_lon: Optional[ci.HelperLatLon] = None,
        new_tw: Optional[Wind] = None,
    ) -> MustChangeReason:
        """Check if the path must be changed.

        Evaluates a prioritized set of conditions and returns a `MustChangeReason` indicating
        whether a path change is required and a reason.

        Priority of checks (first matching condition wins):
        - Receipt of a new global path or target waypoint (always requires a new local path)
        - Missing OMPL path, LocalPathState, or local path
        - Current path intersects a collision zone
        - Path time-to-live (TTL) has expired
        - Significant wind change between the rolling wind average and the wind used to
          generate the current path (only evaluated when the rolling average is available
          and the current path was not generated from a single true-wind point)
        - Invalid target local waypoint index (too low or beyond available waypoints)
        - Boat has deviated from the current path segment beyond the allowed threshold

        Args:
            received_new_global_waypoint (bool): True when Sailbot adopted a new global path or
                advanced to a new target waypoint, so local-path regeneration should be triggered.
            boat_lat_lon (Optional[ci.HelperLatLon], optional): Current boat position used to
                evaluate segment deviation. If None, deviation is not evaluated.
            new_tw (Optional[Wind], optional): The most recent true wind reading. This
                value is used to update wind history before evaluating wind-based switching.

        Returns:
            MustChangeReason
        """
        if received_new_global_waypoint:
            return MustChangeReason(True, "Received new global waypoint")
        if self._ompl_path is None:
            return MustChangeReason(True, "OMPL path is None")
        if self.state is None:
            return MustChangeReason(True, "State is None")
        if self.path is None:
            return MustChangeReason(True, "Path is None")
        if self.in_collision_zone():
            return MustChangeReason(True, "Path intersects collision zone")
        if self.is_path_expired():
            return MustChangeReason(True, "Path has expired (TTL exceeded)")
        if new_tw is None:
            self._logger.debug("Wind condition not met: new_tw is None")
        elif self.state.wind_tracker.tw_avg is None:
            self._logger.debug("Wind condition not met: wind_tracker.tw_avg is None")
        elif self.state.path_generated_wind is None:
            self._logger.debug("Wind condition not met: state.path_generated_wind is None")
        # disabled for testing
        # elif self.state.wind_tracker.using_one_tw_point:
        #   self._logger.debug("Wind condition not met: wind_tracker.using_one_tw_point is True")
        elif self.is_significant_wind_change(
            self.state.wind_tracker.tw_avg, self.state.path_generated_wind
        ):
            return MustChangeReason(True, "Significant wind change")
        if self._target_lp_wp_index < 1:
            return MustChangeReason(
                True, f"Target waypoint index too low: {self._target_lp_wp_index}"
            )
        if self._target_lp_wp_index >= len(self.path.waypoints):
            return MustChangeReason(
                True,
                f"Target waypoint index out of bounds: {self._target_lp_wp_index} >= {len(self.path.waypoints)}",  # noqa
            )
        if boat_lat_lon is not None and self.exceeded_segment_deviation(
            self.path,
            self._target_lp_wp_index,
            boat_lat_lon,
        ):
            return MustChangeReason(True, "Boat deviated from path segment")

        return MustChangeReason(False, "Path is valid, no change needed")

    def update_if_needed(
        self,
        inputs: LocalPathInputs,
        target_lp_wp_index: int,
        received_new_global_waypoint: bool,
    ) -> tuple[float, int]:
        """Updates the local path using OMPL if conditions warrant a path change.

        Converts apparent wind to true wind, updates the rolling true wind tracker, then
        evaluates whether to update the current path based on several criteria:
        - Receipt of a new global path or target waypoint
        - Absence of an existing OMPL path, local path, or state
        - Current path intersecting with collision zones
        - Current path exceeding its time-to-live
        - Significant change between current rolling wind and path-generation wind
        - Invalid target local waypoint index

        Args:
            inputs (LocalPathInputs): Current navigation inputs used to update or regenerate
                the local path.
            target_lp_wp_index (int): 0-based array index of the local waypoint Polaris is
                currently heading toward. This starts at index 1 because OMPL index 0 is the
                start state near the boat.
            received_new_global_waypoint (bool): Flag indicating that the active global path or
                target global waypoint changed and the local path must be regenerated.

        Returns:
            tuple[float, int]: A tuple containing:
                - Desired heading in degrees
                - Updated waypoint index
            The method decides whether to return the heading for a newly generated path or the
            existing path.

        Raises:
            PathNotFoundError: If a required path is unavailable or a replacement path cannot be
                generated.
            ValueError: If the LocalPathState is passed invalid values.
            IndexError: If the target waypoint is reached and no next local waypoint exists.
        """
        self._target_lp_wp_index = target_lp_wp_index
        boat_lat_lon = None

        if inputs.filtered_wind_sensor is None:
            raise PathNotFoundError("filtered_wind_sensor is None")
        if inputs.heading is None:
            raise PathNotFoundError("heading is None")

        # Convert apparent wind to true wind
        new_aw = Wind(
            inputs.filtered_wind_sensor.speed.speed,
            inputs.filtered_wind_sensor.direction,
        )
        new_heading = inputs.heading.heading
        new_tw = None
        if inputs.gps is not None:
            new_aw_dir_deg_gc = boat_to_global_coordinate(new_heading, new_aw.dir_deg)
            tw_dir_deg, tw_speed_kmph = aw_gc_to_tw_gc(
                aw_dir_deg_gc=new_aw_dir_deg_gc,
                aw_speed_kmph=new_aw.speed_kmph,
                boat_heading_deg_gc=new_heading,
                boat_speed_kmph=inputs.gps.speed.speed,
            )
            new_tw = Wind(tw_speed_kmph, tw_dir_deg)

        if self.state and self.state.wind_tracker and new_tw is not None:
            self.state.wind_tracker.update_tw_history(new_tw)

        if self.state:
            try:
                self.state.update_state(  # type: ignore
                    inputs.gps,
                    inputs.heading,
                    inputs.ais_ships,
                    inputs.filtered_wind_sensor,
                )
                self.state.update_obstacles()
                boat_lat_lon = inputs.gps.lat_lon
            except ValueError as e:
                # No need to handle anything else here. There is no compulsion for the path to
                # change so an improper state can be ignored. While not ideal, this is better than
                # stopping the boat.
                self._logger.warn(f"State update did not complete: {e}")
                boat_lat_lon = self.state.position  # type: ignore

        must_change_reason = self.must_change_path(
            received_new_global_waypoint,
            boat_lat_lon,
            new_tw,
        )

        if must_change_reason.should_change_path:
            tries = 0
            new_ompl_path = None
            new_state = None
            while tries < MAX_OMPL_PATH_GEN_TRIES:
                try:
                    if self.state is None or self.state.wind_tracker is None:
                        wind_tracker = WindTracker()
                        if new_tw is not None:
                            wind_tracker.update_tw_history(new_tw)
                    else:
                        wind_tracker = self.state.wind_tracker
                    new_state = LocalPathState(
                        gps=inputs.gps,
                        heading=inputs.heading,
                        ais_ships=inputs.ais_ships,
                        global_path=inputs.global_path,
                        target_global_waypoint=inputs.target_global_waypoint,
                        filtered_wind_sensor=inputs.filtered_wind_sensor,
                        wind_tracker=wind_tracker,
                        path_generated_time_sec=self._now_sec(),
                    )
                    new_ompl_path = OMPLPath(
                        parent_logger=self._logger,
                        local_path_state=new_state,
                        land_multi_polygon=inputs.land_multi_polygon,
                    )
                    if new_ompl_path.solved:
                        break
                    else:
                        self._logger.warn(
                            f"OMPL path generation attempt {tries + 1}/{MAX_OMPL_PATH_GEN_TRIES} "
                            "did not solve"
                        )
                        tries += 1
                except ValueError as e:
                    self._logger.warn(
                        f"OMPL path generation attempt {tries + 1}/{MAX_OMPL_PATH_GEN_TRIES} "
                        f"raised ValueError: {e}"
                    )
                    tries += 1

            if not new_ompl_path or not new_ompl_path.solved:
                # We failed to generate a new path after several tries,
                # but the old path is also not valid anymore.
                if new_state is not None:
                    new_state.wind_tracker.using_one_tw_point = (
                        new_state.wind_tracker.tw_avg is None
                    )
                    self.state = new_state
                self._logger.warn(
                    "Old Path must change and new path couldn't be solved"
                    + f" within {MAX_OMPL_PATH_GEN_TRIES}"
                )
                raise PathNotFoundError(
                    "Old Path must change and new path couldn't be solved"
                    + f" within {MAX_OMPL_PATH_GEN_TRIES}"
                )

            if self.state is not None:
                time_on_prev_sec = self._now_sec() - self.state.path_generated_time_sec
                self._logger.info(
                    f"Previous local path was active for {time_on_prev_sec:.1f}s before switching"
                )
            self._logger.info(f"Updating local path: {must_change_reason.reason}")
            self.last_remaining_waypoints = self._count_remaining_waypoints()
            self.last_replan_reason = must_change_reason.reason
            # Record whether this newly accepted path was generated before the wind average
            # was available, since the tracker is shared across path states.
            wind_tracker = new_state.wind_tracker  # type: ignore[union-attr]
            wind_tracker.using_one_tw_point = wind_tracker.tw_avg is None
            self.state = new_state
            self._update(new_ompl_path)

            init_target_lp_wp_index = 1
            try:
                heading_new_path, new_target_lp_wp_index = (
                    self.calculate_desired_heading_and_wp_index(
                        new_ompl_path.get_path(), init_target_lp_wp_index, inputs.gps.lat_lon
                    )
                )
            except IndexError:
                self._logger.warn("New Path's desired heading index update failed")
                raise PathNotFoundError("New Path's desired heading index update failed")

            return heading_new_path, new_target_lp_wp_index

        self.last_replan_reason = ""
        self.last_remaining_waypoints = 0

        try:
            self._logger.info(f"Reusing local path: {must_change_reason.reason}")
            heading_old_path, old_target_lp_wp_index = self.calculate_desired_heading_and_wp_index(
                self._ompl_path.get_path(), target_lp_wp_index, boat_lat_lon  # type: ignore
            )
        except IndexError:
            self._logger.warn("Current Path's desired heading index update failed")
            raise PathNotFoundError("Current Path's desired heading index update failed")
        return heading_old_path, old_target_lp_wp_index

    def _update(self, ompl_path: OMPLPath):

        self._ompl_path = ompl_path
        self.path = self._ompl_path.get_path()
