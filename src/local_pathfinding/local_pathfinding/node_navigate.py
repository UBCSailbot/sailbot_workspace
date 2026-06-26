"""The main node of the local_pathfinding package, represented by the `Sailbot` class."""

import csv
import math
import os
import tempfile
import traceback

import rclpy
from rclpy.node import Node
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
from local_pathfinding.local_path import LocalPath, LocalPathInputs, PathNotFoundError
from local_pathfinding.ompl_path import MAX_SOLVER_RUN_TIME_SEC

GLOBAL_WAYPOINT_REACHED_THRESH_M = 300
GPS_TIMEOUT_SEC = 120.0
WAYPOINT_EQUAL_ABS_TOL_DEG = 1e-7
NANOSEC_PER_SEC = 1_000_000_000
MAIN_GP_FILE_PATH = "/workspaces/sailbot_workspace/src/local_pathfinding/local_pathfinding/global_path_storage/main_global_path.csv"  # noqa
BACKUP_GP_FILE_PATH = "/workspaces/sailbot_workspace/src/local_pathfinding/local_pathfinding/global_path_storage/backup_global_path.csv"  # noqa

"""
GlobalPath intentionally stays small: it is only the waypoint list, the current waypoint index, and
whether the path came from backup storage. Sailbot owns the higher-level policy around when a path
is accepted, when sail should be disabled, and when the local planner needs to replan.

When a new global path comes from NET, we trust the waypoint ordering from network systems:
index 0 is the final destination and len(waypoints) - 1 is the first target. Sailbot persists that
incoming path to MAIN_GP_FILE_PATH before adopting it in memory, so a path that cannot be saved
does not silently become the active route.

NET may republish the same global path repeatedly. Once that path is already the active main path,
Sailbot treats repeated identical waypoint lists as heartbeat-style duplicates: it preserves the
current index and does not signal a new local-path replan.

If accepting the NET path fails, Sailbot falls back to persisted route data. It tries the persisted
main path first, then BACKUP_GP_FILE_PATH. Persisted paths need GPS because the boat may have moved
since the route was saved, and restarting at len(waypoints) - 1 could send it backward after a
power cycle. Without GPS, or without any usable path, Sailbot leaves gp unset and the existing
inactive-input path publishes sail=False.

This assumes persisted main and backup paths use the same reverse-order convention as NET paths.
On resume, Sailbot finds the closest non-final waypoint and targets one index closer to the final
destination. For example, if the closest waypoint is index 6, the resumed target is index 5. This
is an intentional simplification: it may skip the closest waypoint when the boat is behind or
between waypoints, but it biases recovery toward continuing down-route to index 0 instead of
backtracking.

Once selected, the path still advances toward the destination by decrementing the index.

Once a path is active, navigation advances by decrementing the index. When the index drops below
zero, the global path is exhausted and Sailbot disables sail instead of cycling between waypoints.

Sailbot keeps a separate received_new_global_path flag as a one-shot bridge from path adoption to
the next desired-heading tick. get_desired_heading() combines that flag with same-tick waypoint
advancement into the received_new_global_waypoint argument passed to LocalPath.update_if_needed().
Same-tick waypoint advancement is committed only after a successful local-path update, so a failed
local plan cannot silently skip ahead through the global path. The bridge flag is cleared only after
a successful local-path update, or when the global path is exhausted and no retryable local-path
update remains.
"""


class GlobalPath:
    """Small navigation state wrapper for a reverse-ordered global path."""

    def __init__(
        self,
        waypoints: list[ci.HelperLatLon],
        index: int,
        is_backup: bool = False,
    ) -> None:
        self.waypoints = waypoints
        self.index = index
        self.is_backup = is_backup

    @property
    def target_waypoint(self) -> ci.HelperLatLon | None:
        """Return the current target waypoint, or None when the path is exhausted."""
        if self.index < 0 or self.index >= len(self.waypoints):
            return None
        return self.waypoints[self.index]

    def advance_waypoint(self) -> bool:
        """Advance to the next global waypoint and return False when exhausted."""
        self.index -= 1
        return self.target_waypoint is not None


def main(args=None) -> None:
    rclpy.init(args=args)
    sailbot = Sailbot()

    rclpy.spin(node=sailbot)

    sailbot.destroy_node()
    rclpy.shutdown()


class Sailbot(Node):
    """Stores, updates, and maintains the state of our autonomous sailboat.

    Subscribers:
        ais_ships_sub (Subscription): Subscribe to a `AISShips` msg.
        gps_sub (Subscription): Subscribe to a `GPS` msg.
        global_path_sub (Subscription): Subscribe to a `Path` msg.
        filtered_wind_sensor_sub (Subscription): Subscribe to a `WindSensor` msg.

    Publishers:
        desired_heading_pub (Publisher): Publish the desired heading in a `DesiredHeading` msg.
        lpath_data_pub (Publisher): Publish all local path data in a `LPathData` msg.

    Publisher timers:
        pub_period_sec (float): The period of the publisher timers.
        desired_heading_timer (Timer): Call the desired heading callback function.

    Attributes from subscribers:
        ais_ships (ci.AISShips): Data from other boats.
        gps (ci.GPS): Data from the GPS sensor.
        filtered_wind_sensor (ci.WindSensor): Filtered data from the wind sensors.
        desired_heading (ci.DesiredHeading): current desired heading.
        target_lp_wp_index (int): 0-based array index of the local waypoint Polaris is currently
            heading toward. This starts at 1 because OMPL index 0 is the start state near the
            boat.

    Attributes:
        local_path (LocalPath): The path that `Sailbot` is following.
        planner (str): The path planner that `Sailbot` is using.
        mode (str): Runtime mode. Development mode publishes richer local path data.
        gp (GlobalPath): Global path state that `Sailbot` is following.
        received_new_global_path (bool): One-shot signal that the active global path changed and
            the next desired-heading tick must force a local path update.
        land_multi_polygon (MultiPolygon): Optional mock land data used in development mode.
    """

    def __init__(self) -> None:
        super().__init__(node_name="navigate")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("mode", rclpy.Parameter.Type.STRING),
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("path_planner", rclpy.Parameter.Type.STRING),
                ("test_plan", rclpy.Parameter.Type.STRING),
                ("config", rclpy.Parameter.Type.STRING),
            ],
        )

        # subscribers
        self.ais_ships_sub = self.create_subscription(
            msg_type=ci.AISShips,
            topic="ais_ships",
            callback=self.ais_ships_callback,
            qos_profile=10,
        )
        self.gps_sub = self.create_subscription(
            msg_type=ci.GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )
        self.global_path_sub = self.create_subscription(
            msg_type=ci.Path,
            topic="global_path",
            callback=self.global_path_callback,
            qos_profile=10,
        )
        self.filtered_wind_sensor_sub = self.create_subscription(
            msg_type=ci.WindSensor,
            topic="filtered_wind_sensor",
            callback=self.filtered_wind_sensor_callback,
            qos_profile=10,
        )

        # publishers
        self.desired_heading_pub = self.create_publisher(
            msg_type=ci.DesiredHeading, topic="desired_heading", qos_profile=10
        )
        self.lpath_data_pub = self.create_publisher(
            msg_type=ci.LPathData, topic="local_path", qos_profile=10
        )

        # publisher timers
        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )
        self.get_logger().debug(f"Got parameter: {self.pub_period_sec=}")

        # we need to give the solver time to run and the callback to return before calling again
        # so we add the solver max allowed runtime to our publishing period
        self.desired_heading_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec + MAX_SOLVER_RUN_TIME_SEC,
            callback=self.desired_heading_callback,
        )

        # attributes from subscribers
        self.ais_ships: ci.AISShips | None = None
        self.gps: ci.GPS | None = None
        self.gp: GlobalPath | None = None
        self.filtered_wind_sensor: ci.WindSensor | None = None
        self.desired_heading: ci.DesiredHeading | None = None

        # attributes
        self.gps_timeout_start_sec = self._now_sec()
        self.local_path = LocalPath(
            parent_logger=self.get_logger(),
            now_sec=self._now_sec,
        )
        self.target_lp_wp_index = 1
        self.received_new_global_path = False
        self._load_persisted_global_path()
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.planner = self.get_parameter("path_planner").get_parameter_value().string_value
        global_config = self.get_parameter("config").get_parameter_value().string_value
        self.get_logger().debug(f"Got parameter: {self.planner=}")

        # Initialize mock land obstacle
        self.land_multi_polygon = None
        if self.mode in ["development", "sim"]:
            self.test_plan = self.get_parameter("test_plan").get_parameter_value().string_value

            if self.test_plan:
                self.get_logger().warn("User has manually overridden test plan through CLI")

            self.get_logger().info("test plan: " + self.test_plan)
            test_plan = TestPlan(self.test_plan)
            self.land_multi_polygon = test_plan.land
            self.get_logger().info("Loaded mock land data.")

        if global_config == "on_water_globals.yaml":
            self.get_logger().info(
                f"On water globals config ({global_config}) is successfully loaded "
            )
        elif global_config == "launch_globals.yaml":
            self.get_logger().info(
                f"Launch globals config ({global_config}) is successfully loaded "
            )
        else:
            self.get_logger().info(
                f"Default globals config ({global_config}) is successfully loaded "
            )

    def _now_sec(self) -> float:
        """Return the current ROS clock time in seconds."""
        return self.get_clock().now().nanoseconds / NANOSEC_PER_SEC

    @staticmethod
    def _path_to_dict(path: ci.Path, num_decimals: int = 4) -> dict[int, str]:
        """Converts a ci.Path msg to a dictionary suitable for logging."""
        return {
            i: f"({waypoint.latitude:.{num_decimals}f}, {waypoint.longitude:.{num_decimals}f})"
            for i, waypoint in enumerate(path.waypoints)
        }

    @staticmethod
    def _same_waypoints(left: list[ci.HelperLatLon], right: list[ci.HelperLatLon]) -> bool:
        """Return whether two waypoint lists describe the same global path."""
        return len(left) == len(right) and all(
            math.isclose(
                left_waypoint.latitude,
                right_waypoint.latitude,
                rel_tol=0.0,
                abs_tol=WAYPOINT_EQUAL_ABS_TOL_DEG,
            )
            and math.isclose(
                left_waypoint.longitude,
                right_waypoint.longitude,
                rel_tol=0.0,
                abs_tol=WAYPOINT_EQUAL_ABS_TOL_DEG,
            )
            for left_waypoint, right_waypoint in zip(left, right)
        )

    def _write_global_path_to_file(self, path: ci.Path) -> None:
        """Writes the global path to the persisted csv file.

        Creates the file if it does not exist. If it already exists, overwrites it with the
        latest global path.

        Raises:
            PermissionError: If the directory or file cannot be written due to insufficient
                permissions.
            IsADirectoryError: If MAIN_GP_FILE_PATH points to a directory.
            NotADirectoryError: If a parent path component is a file instead of a directory.
            OSError: If another filesystem error occurs while creating the directory, opening
                the file, or writing the file.
            AttributeError: If path or one of its waypoints does not have the expected
                attributes.


        TODO: VERIFY THAT ALL THE FILES ARE WRITABLE ON THE PI
        """
        parent_dir = os.path.dirname(MAIN_GP_FILE_PATH)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)

        if os.path.exists(MAIN_GP_FILE_PATH):
            self.get_logger().debug(f"Overwriting global path file: {MAIN_GP_FILE_PATH}")
        else:
            self.get_logger().debug(f"Creating global path file: {MAIN_GP_FILE_PATH}")

        temp_file_path = None
        try:
            with tempfile.NamedTemporaryFile(
                "w",
                dir=parent_dir,
                delete=False,
                newline="",
            ) as file:
                temp_file_path = file.name
                writer = csv.writer(file)
                writer.writerow(["latitude", "longitude"])
                for waypoint in path.waypoints:
                    writer.writerow([waypoint.latitude, waypoint.longitude])
            os.replace(temp_file_path, MAIN_GP_FILE_PATH)
        except Exception:
            if temp_file_path is not None and os.path.exists(temp_file_path):
                try:
                    os.remove(temp_file_path)
                except OSError:
                    pass
            raise
        self.get_logger().debug(f"writing path to file: {MAIN_GP_FILE_PATH} successful!!")

    @staticmethod
    def _read_global_path_from_file(file_path: str) -> ci.Path:
        """Reads a global path from a csv file.

        Raises:
            FileNotFoundError: If file_path does not exist.
            PermissionError: If file_path cannot be read due to insufficient permissions.
            IsADirectoryError: If file_path points to a directory.
            NotADirectoryError: If a parent path component is a file instead of a directory.
            OSError: If another filesystem error occurs while opening or reading the file.
            StopIteration: If the csv file is empty and does not contain a header row.
            IndexError: If a waypoint row does not contain both latitude and longitude columns.
            ValueError: If a waypoint latitude or longitude cannot be converted to a float.
        """
        path = ci.Path()

        with open(file_path, "r") as file:
            reader = csv.reader(file)
            reader.__next__()
            for row in reader:
                path.waypoints.append(
                    ci.HelperLatLon(latitude=float(row[0]), longitude=float(row[1]))
                )
        return path

    def _path_from_gp(self) -> ci.Path:
        """Return the current global path state as a ROS Path message."""
        if self.gp is None:
            return ci.Path()
        return ci.Path(waypoints=list(self.gp.waypoints))

    def _create_gp(
        self,
        path: ci.Path,
        is_backup: bool,
        is_new_global_path: bool = False,
    ) -> GlobalPath | None:
        """Create GlobalPath state from a ROS path, choosing the correct starting index."""
        if not path.waypoints:
            return None

        if not is_new_global_path or is_backup:
            if self.gps is None:
                self.get_logger().warning(
                    "Global path is available, but GPS is unavailable. "
                    "Waiting for GPS before loading persisted path."
                )
                return None

            index = self._resume_waypoint_index(path, self.gps.lat_lon)
            path_source = "backup" if is_backup else "main"
            self.get_logger().info(
                f"Using persisted {path_source} global path. "
                f"Starting at resume waypoint index: {index}"
            )
        else:
            index = len(path.waypoints) - 1

        return GlobalPath(waypoints=list(path.waypoints), index=index, is_backup=is_backup)

    def _set_gp(self, gp: GlobalPath) -> None:
        """Store a new global path and signal that the local planner must replan."""
        self.gp = gp
        self.received_new_global_path = True

    def _load_persisted_global_path(self) -> bool:
        """Load the latest persisted global path or static backup path if available."""

        read_errors = (
            FileNotFoundError,
            PermissionError,
            IsADirectoryError,
            NotADirectoryError,
            OSError,
            StopIteration,
            IndexError,
            ValueError,
        )

        for file_path in [MAIN_GP_FILE_PATH, BACKUP_GP_FILE_PATH]:
            try:
                path = self._read_global_path_from_file(file_path)
                if not path.waypoints:
                    raise ValueError("global path csv has no waypoints")

                gp = self._create_gp(
                    path,
                    is_backup=file_path == BACKUP_GP_FILE_PATH,
                )
                if gp is None:
                    continue

                self._set_gp(gp)
                self.get_logger().info(f"Loaded global path from {file_path}")
                return True
            except read_errors as err:
                log = (
                    self.get_logger().debug
                    if file_path == MAIN_GP_FILE_PATH and isinstance(err, FileNotFoundError)
                    else self.get_logger().warning
                )
                log(f"Failed to load global path from {file_path}: {err}")

        self.get_logger().warning(
            "No persisted global path could be loaded. Waiting for network systems to "
            "publish global_path."
        )
        return False

    # subscriber callbacks
    def ais_ships_callback(self, msg: ci.AISShips) -> None:
        self.get_logger().debug(f"Received data from {self.ais_ships_sub.topic}: {msg}")
        self.ais_ships = msg

    def gps_callback(self, msg: ci.GPS) -> None:
        self.get_logger().debug(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg
        self.gps_timeout_start_sec = self._now_sec()

    def global_path_callback(self, msg: ci.Path) -> None:
        self.get_logger().debug(
            f"Received data from {self.global_path_sub.topic}: {self._path_to_dict(msg)}"
        )
        if not msg.waypoints:
            self.get_logger().warning(
                "Received empty global path. Keeping current in-memory path if available "
                "and trying persisted fallback."
            )
            if self.gp is None:
                self._load_persisted_global_path()
            return

        if (
            self.gp is not None
            and not self.gp.is_backup
            and self._same_waypoints(self.gp.waypoints, list(msg.waypoints))
        ):
            self.get_logger().debug("Received unchanged global path. Keeping current progress.")
            return

        try:
            self._write_global_path_to_file(msg)
        except (OSError, AttributeError) as err:
            self.get_logger().error(
                f"Failed to persist global path to {MAIN_GP_FILE_PATH}: {err}. "
                "Trying persisted fallback."
            )
            self._load_persisted_global_path()
            return

        gp = self._create_gp(msg, is_backup=False, is_new_global_path=True)
        if gp is not None:
            self._set_gp(gp)

    def filtered_wind_sensor_callback(self, msg: ci.WindSensor) -> None:
        self.get_logger().debug(f"Received data from {self.filtered_wind_sensor_sub.topic}: {msg}")
        self.filtered_wind_sensor = msg

    # publisher callbacks
    def desired_heading_callback(self) -> None:
        """Get and publish the desired heading."""

        if self.gp is None:
            self._load_persisted_global_path()

        if self._has_gps_timed_out():
            msg = ci.DesiredHeading()
            msg.heading.heading = 0.0
            msg.sail = False

            self.desired_heading = msg

            self.get_logger().warning(
                f"GPS data has not been received for more than {GPS_TIMEOUT_SEC:.0f} seconds. "
                f"Publishing to {self.desired_heading_pub.topic}: {msg.heading.heading}"
            )
            self.desired_heading_pub.publish(msg)
            return  # should not continue, try again next loop

        if not self._all_subs_active():
            self._log_inactive_subs_warning()
            msg = ci.DesiredHeading()
            msg.heading.heading = 0.0
            msg.sail = False
            self.desired_heading = msg
            self.get_logger().debug(
                f"Publishing to {self.desired_heading_pub.topic}: {msg.heading.heading}"
            )
            self.desired_heading_pub.publish(msg)
            return  # should not continue, return and try again next loop

        try:
            self.update_params()

            desired_heading, sail = self.get_desired_heading()
            msg = ci.DesiredHeading()
            msg.heading.heading = desired_heading
            msg.sail = sail
            if (
                self.desired_heading is None
                or desired_heading != self.desired_heading.heading.heading
            ):
                self.get_logger().info(f"Updating desired heading to: {msg.heading.heading:.2f}")

            self.desired_heading = msg

            self.get_logger().info(
                f"Publishing to {self.desired_heading_pub.topic}: {msg.heading.heading}, "
                f"sail == {msg.sail}"  # noqa
            )
            self.desired_heading_pub.publish(msg)

            self.get_logger().debug(f"Publishing local path data to {self.lpath_data_pub.topic}")
            self.publish_local_path_data(msg.sail)
        except Exception:
            self.get_logger().error(
                "Unexpected error in the pathfinding loop. "
                f"gps_lat_lon={self.gps.lat_lon if self.gps is not None else None}, "
                f"target_gp_waypoint={self.gp.target_waypoint if self.gp is not None else None}, "
                f"gp_index={self.gp.index if self.gp is not None else None}, "
                f"target_lp_wp_index={self.target_lp_wp_index}, planner={self.planner}\n"
                f"{traceback.format_exc()}"
            )
            raise

    def publish_local_path_data(self, sail: bool) -> None:
        """
        Collect all navigation data and publish it in one message.
        In development and sim modes, all navigation data is published.
        In production mode, only the local path is published.

        """

        # When sail is disabled the planner has no valid local path to follow; publish an empty
        # Path rather than a stale or None one, so downstream consumers
        # (e.g. the visualizer and website) render the "no local path" state
        # instead of crashing on a None / misleading submessage.
        local_path = (
            self.local_path.path if (sail and self.local_path.path is not None) else ci.Path()
        )

        # publish all navigation data when in dev mode
        if self.mode in ["development", "sim"]:
            helper_obstacles = []

            # state is None until the first successful local path. On a sail-disabled failure it
            # may still hold the obstacles that caused the failure, so iterate only if present.
            state = self.local_path.state
            if state is not None:
                for obst in state.obstacles:

                    if isinstance(obst, ob.Land):
                        for polygon in obst.collision_zone.geoms:
                            latlon_polygon = cs.xy_polygon_to_latlon_polygon(
                                state.reference_latlon, polygon
                            )

                            # each point of the polygon is in lat lon now
                            # but you can't construct a Shapely polygon out of HelperLatLon objects
                            # so each point is a shapely Point that needs to be converted to a
                            # HelperLatLon, before it can be published to ROS
                            helper_latlons = [
                                ci.HelperLatLon(longitude=point[0], latitude=point[1])
                                for point in latlon_polygon.exterior.coords
                            ]
                            helper_obstacles.append(
                                ci.HelperObstacle(points=helper_latlons, obstacle_type="Land")
                            )
                    else:  # is a Boat
                        latlon_polygon = cs.xy_polygon_to_latlon_polygon(
                            state.reference_latlon, obst.collision_zone
                        )
                        helper_latlons = [
                            ci.HelperLatLon(longitude=point[0], latitude=point[1])
                            for point in latlon_polygon.exterior.coords
                        ]
                        helper_obstacles.append(
                            ci.HelperObstacle(points=helper_latlons, obstacle_type="Boat")
                        )

            msg = ci.LPathData(
                global_path=self._path_from_gp(),
                local_path=local_path,
                gps=self.gps,
                filtered_wind_sensor=self.filtered_wind_sensor,
                ais_ships=self.ais_ships,
                obstacles=helper_obstacles,
                desired_heading=self.desired_heading,
                replan_reason=self.local_path.last_replan_reason,
                remaining_waypoints=self.local_path.last_remaining_waypoints,
            )
        else:
            # in production only publish the local path for website
            msg = ci.LPathData(local_path=local_path)

        self.lpath_data_pub.publish(msg)

    # helper functions
    def get_desired_heading(self) -> tuple[float, bool]:
        """Get the desired heading.

        Returns:
            tuple[float, bool]: The desired heading and whether sailing is allowed. Sailing is
            disabled when no local path can be generated.
        """

        if self.gp is None:
            self.get_logger().info("No global path is available; disabling sail")
            self.local_path.path = ci.Path(waypoints=[])
            return 0.0, False
        if self.gps is None:
            self.get_logger().info("No GPS is available; disabling sail")
            self.local_path.path = ci.Path(waypoints=[])
            return 0.0, False

        target_global_waypoint = self.gp.target_waypoint
        if target_global_waypoint is None:
            self.get_logger().info("Global path is exhausted; disabling sail")
            self.received_new_global_path = False
            self.local_path.path = ci.Path(waypoints=[])
            return 0.0, False

        self.get_logger().info(
            f"Current target global waypoint: {target_global_waypoint} (index: {self.gp.index})"
        )

        # Check if we're close enough to the global waypoint to head to the next one
        _, _, distance_to_waypoint_m = cs.GEODESIC.inv(
            self.gps.lat_lon.longitude,
            self.gps.lat_lon.latitude,
            target_global_waypoint.longitude,
            target_global_waypoint.latitude,
        )

        received_new_global_waypoint = self.received_new_global_path
        original_gp_index = self.gp.index
        if distance_to_waypoint_m < GLOBAL_WAYPOINT_REACHED_THRESH_M:
            received_new_global_waypoint = True
            if not self.gp.advance_waypoint():
                self.get_logger().info("Reached final global waypoint; disabling sail")
                self.received_new_global_path = False
                self.local_path.path = ci.Path(waypoints=[])
                return 0.0, False
            target_global_waypoint = self.gp.target_waypoint

        try:
            desired_heading, self.target_lp_wp_index = self.local_path.update_if_needed(
                inputs=LocalPathInputs(
                    gps=self.gps,
                    ais_ships=self.ais_ships,
                    global_path=self._path_from_gp(),
                    target_global_waypoint=target_global_waypoint,
                    filtered_wind_sensor=self.filtered_wind_sensor,
                    planner=self.planner,
                    land_multi_polygon=self.land_multi_polygon,
                ),
                target_lp_wp_index=self.target_lp_wp_index,
                received_new_global_waypoint=received_new_global_waypoint,
            )

            local_target_wp = None
            if self.local_path.path is not None and (
                0 <= self.target_lp_wp_index < len(self.local_path.path.waypoints)
            ):
                local_target_wp = self.local_path.path.waypoints[self.target_lp_wp_index]
            self.get_logger().info(
                f"Current target local waypoint: {local_target_wp} "
                f"(index {self.target_lp_wp_index})"
            )

            self.received_new_global_path = False
            return desired_heading, True
        except PathNotFoundError:
            self.gp.index = original_gp_index
            self.received_new_global_path = received_new_global_waypoint
            self.get_logger().warning("Unable to generate a local path; disabling sail")
            self.local_path.path = ci.Path(waypoints=[])
            return 0.0, False

    @staticmethod
    def _resume_waypoint_index(global_path: ci.Path, lat_lon: ci.HelperLatLon) -> int:
        """Return a resume index biased one waypoint toward the final destination."""

        min_i = len(global_path.waypoints) - 1
        min_distance_m = float("inf")
        for i in range(len(global_path.waypoints) - 1, 0, -1):
            distance_m = cs.GEODESIC.inv(
                lat_lon.longitude,
                lat_lon.latitude,
                global_path.waypoints[i].longitude,
                global_path.waypoints[i].latitude,
            )[2]

            if min_distance_m > distance_m:
                min_distance_m = distance_m
                min_i = i

        return max(min_i - 1, 0)

    def update_params(self) -> None:
        """Update instance variables that depend on parameters if they have changed."""

        mode = self.get_parameter("mode").get_parameter_value().string_value
        if mode != self.mode:
            self.get_logger().debug(f"switching from {self.mode} mode to {mode} mode")
            self.mode = mode

        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        if pub_period_sec != self.pub_period_sec:
            self.get_logger().debug(
                f"Updating pub period and timer from {self.pub_period_sec} to {pub_period_sec}"
            )
            self.pub_period_sec = pub_period_sec
            self.desired_heading_timer.cancel()
            self.desired_heading_timer = self.create_timer(
                timer_period_sec=self.pub_period_sec + MAX_SOLVER_RUN_TIME_SEC,
                callback=self.desired_heading_callback,
            )

        planner = self.get_parameter("path_planner").get_parameter_value().string_value
        if planner != self.planner:
            self.get_logger().debug(f"Updating planner from {self.planner} to {planner}")
            self.planner = planner

    def _all_subs_active(self) -> bool:
        return (
            self.ais_ships is not None
            and self.gps is not None
            and self.gp is not None
            and self.filtered_wind_sensor is not None
        )

    def _has_gps_timed_out(self) -> bool:
        """Checks if we haven't received a GPS message for more than 2 minutes."""

        elapsed_sec = self._now_sec() - self.gps_timeout_start_sec
        return elapsed_sec > GPS_TIMEOUT_SEC

    def _log_inactive_subs_warning(self) -> None:
        """
        Logs a warning message for each inactive subscriber.
        """
        inactive_subs = []
        if self.ais_ships is None:
            inactive_subs.append("ais_ships")
        if self.gps is None:
            inactive_subs.append("gps")
        if self.gp is None:
            inactive_subs.append("global_path")
        if self.filtered_wind_sensor is None:
            inactive_subs.append("filtered_wind_sensor")
        if len(inactive_subs) == 0:
            return
        self.get_logger().warning(
            "Missing navigation inputs: "
            + ", ".join(inactive_subs)
            + "; publishing desired heading with sail disabled"
        )


if __name__ == "__main__":
    main()
