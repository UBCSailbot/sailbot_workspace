"""The main node of the local_pathfinding package, represented by the `Sailbot` class."""

import traceback

import rclpy
from rclpy.node import Node
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs
import local_pathfinding.global_path as gp
import local_pathfinding.obstacles as ob
from local_pathfinding.local_path import LocalPath, LocalPathInputs, PathNotFoundError
from local_pathfinding.ompl_path import MAX_SOLVER_RUN_TIME_SEC

GLOBAL_WAYPOINT_REACHED_THRESH_M = 300
REALLY_FAR_M = 100000000
GPS_TIMEOUT_SEC = 120.0
NANOSEC_PER_SEC = 1_000_000_000


def main(args=None):
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
        global_path (ci.Path): Path that we are following.
        filtered_wind_sensor (ci.WindSensor): Filtered data from the wind sensors.
        desired_heading (ci.DesiredHeading): current desired heading.
        target_lp_wp_index (int): 0-based array index of the local waypoint Polaris is currently
            heading toward. This starts at 1 because OMPL index 0 is the start state near the
            boat.

    Attributes:
        local_path (LocalPath): The path that `Sailbot` is following.
        planner (str): The path planner that `Sailbot` is using.
        mode (str): Runtime mode. Development mode publishes richer local path data.
        global_waypoint_index (int): Index of the current target in the reverse-ordered global
            path.
        saved_target_global_waypoint (ci.HelperLatLon): Current global waypoint target.
        land_multi_polygon (MultiPolygon): Optional mock land data used in development mode.
    """

    def __init__(self):
        super().__init__(node_name="navigate")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("mode", rclpy.Parameter.Type.STRING),
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("path_planner", rclpy.Parameter.Type.STRING),
                ("test_plan", rclpy.Parameter.Type.STRING),
                ("global_path_interval_spacing_km", rclpy.Parameter.Type.DOUBLE),
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
        self.global_path_interval_spacing_km = (
            self.get_parameter("global_path_interval_spacing_km")
            .get_parameter_value()
            .double_value
        )
        self.get_logger().debug(f"Got parameter: {self.pub_period_sec=}")

        # we need to give the solver time to run and the callback to return before calling again
        # so we add the solver max allowed runtime to our publishing period
        self.desired_heading_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec + MAX_SOLVER_RUN_TIME_SEC,
            callback=self.desired_heading_callback,
        )

        # attributes from subscribers
        self.ais_ships = None
        self.gps = None
        self.global_path = None
        self.filtered_wind_sensor = None
        self.desired_heading = None

        # attributes
        self.gps_timeout_start_sec = self._now_sec()
        self.local_path = LocalPath(
            parent_logger=self.get_logger(),
            now_sec=self._now_sec,
        )
        self.target_lp_wp_index = 1
        self.global_waypoint_index = -1
        self.saved_target_global_waypoint = None
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.planner = self.get_parameter("path_planner").get_parameter_value().string_value
        self.get_logger().debug(f"Got parameter: {self.planner=}")

        # Initialize mock land obstacle
        self.land_multi_polygon = None
        if self.mode in ["development", "sim"]:
            self.test_plan = self.get_parameter("test_plan").get_parameter_value().string_value
            self.get_logger().info("test plan: " + self.test_plan)
            test_plan = TestPlan(self.test_plan)
            self.land_multi_polygon = test_plan.land
            self.get_logger().info("Loaded mock land data.")

    def _now_sec(self) -> float:
        """Return the current ROS clock time in seconds."""
        return self.get_clock().now().nanoseconds / NANOSEC_PER_SEC

    # subscriber callbacks
    def ais_ships_callback(self, msg: ci.AISShips):
        self.get_logger().debug(f"Received data from {self.ais_ships_sub.topic}: {msg}")
        self.ais_ships = msg

    def gps_callback(self, msg: ci.GPS):
        self.get_logger().debug(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg
        self.gps_timeout_start_sec = self._now_sec()

    def global_path_callback(self, msg: ci.Path):
        self.get_logger().debug(
            f"Received data from {self.global_path_sub.topic}: {gp.path_to_dict(msg)}"
        )
        self.global_path = msg
        if self.saved_target_global_waypoint is None:
            self.saved_target_global_waypoint = self.global_path.waypoints[-1]

    def filtered_wind_sensor_callback(self, msg: ci.WindSensor):
        self.get_logger().debug(f"Received data from {self.filtered_wind_sensor_sub.topic}: {msg}")
        self.filtered_wind_sensor = msg

    # publisher callbacks
    def desired_heading_callback(self):
        """Get and publish the desired heading."""

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
            # Unexpected error in the pathfinding/publish path: log the cause and the inputs in
            # flight, fail safe (sail disabled), and keep the node alive.
            self.get_logger().error(
                "Unexpected error in the pathfinding loop; disabling sail and continuing. "
                f"gps_lat_lon={self.gps.lat_lon if self.gps is not None else None}, "
                f"target_global_waypoint={self.saved_target_global_waypoint}, "
                f"global_waypoint_index={self.global_waypoint_index}, "
                f"target_lp_wp_index={self.target_lp_wp_index}, planner={self.planner}\n"
                f"{traceback.format_exc()}"
            )
            msg = ci.DesiredHeading()
            msg.heading.heading = 0.0
            msg.sail = False
            self.desired_heading = msg
            self.desired_heading_pub.publish(msg)

    def publish_local_path_data(self, sail: bool):
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
            obstacles = self.local_path.state.obstacles if self.local_path.state else []
            for obst in obstacles:

                if isinstance(obst, ob.Land):
                    for polygon in obst.collision_zone.geoms:
                        latlon_polygon = cs.xy_polygon_to_latlon_polygon(
                            self.local_path.state.reference_latlon, polygon
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
                        self.local_path.state.reference_latlon, obst.collision_zone
                    )
                    helper_latlons = [
                        ci.HelperLatLon(longitude=point[0], latitude=point[1])
                        for point in latlon_polygon.exterior.coords
                    ]
                    helper_obstacles.append(
                        ci.HelperObstacle(points=helper_latlons, obstacle_type="Boat")
                    )

            msg = ci.LPathData(
                global_path=self.global_path,
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

        # Extra logic for when the global waypoint changes due to receiving a new global path
        received_new_global_waypoint = False
        if (
            self.global_path.waypoints[self.global_waypoint_index]
            != self.saved_target_global_waypoint
        ):
            received_new_global_waypoint = True
            self.global_waypoint_index = self.determine_start_point_in_new_global_path(
                self.global_path, self.gps.lat_lon, self.global_path_interval_spacing_km
            )
            self.saved_target_global_waypoint = self.global_path.waypoints[
                self.global_waypoint_index
            ]

        self.get_logger().info(
            f"Current target global waypoint: {self.saved_target_global_waypoint}"
            + f"(index {self.global_waypoint_index})"
        )

        # Check if we're close enough to the global waypoint to head to the next one
        _, _, distance_to_waypoint_m = cs.GEODESIC.inv(
            self.gps.lat_lon.longitude,
            self.gps.lat_lon.latitude,
            self.saved_target_global_waypoint.longitude,
            self.saved_target_global_waypoint.latitude,
        )
        if distance_to_waypoint_m < GLOBAL_WAYPOINT_REACHED_THRESH_M:
            received_new_global_waypoint = True
            self.global_waypoint_index -= 1  # Since global waypoints are in reverse order

            # At the end of the global path, alternate between the last two waypoints
            if self.global_waypoint_index < 0:
                self.global_waypoint_index = 1

            self.saved_target_global_waypoint = self.global_path.waypoints[
                self.global_waypoint_index
            ]

        try:
            desired_heading, self.target_lp_wp_index = self.local_path.update_if_needed(
                inputs=LocalPathInputs(
                    gps=self.gps,
                    ais_ships=self.ais_ships,
                    global_path=self.global_path,
                    target_global_waypoint=self.saved_target_global_waypoint,
                    filtered_wind_sensor=self.filtered_wind_sensor,
                    planner=self.planner,
                    land_multi_polygon=self.land_multi_polygon,
                ),
                target_lp_wp_index=self.target_lp_wp_index,
                received_new_global_waypoint=received_new_global_waypoint,
            )
            return desired_heading, True
        except PathNotFoundError:
            self.get_logger().warning("Unable to generate a local path; disabling sail")
            self.local_path.path = ci.Path(waypoints=[])
            return 0.0, False

    @staticmethod
    def determine_start_point_in_new_global_path(
        global_path: ci.Path, boat_lat_lon: ci.HelperLatLon, global_path_spacing_km: float
    ):
        """Used when we receive a new global path.
        Finds the index of the first waypoint within 'pathfinding range' of gps location.
        If none are found, it returns the index of the nearest waypoint."""

        closest_m, index_of_closest = REALLY_FAR_M, -1
        for waypoint_index in range(len(global_path.waypoints) - 1, -1, -1):
            # Note: the global waypoints are in reverse order (index 0 is final waypoint)
            waypoint = global_path.waypoints[waypoint_index]

            _, _, distance_to_waypoint_m = cs.GEODESIC.inv(
                boat_lat_lon.longitude,
                boat_lat_lon.latitude,
                waypoint.longitude,
                waypoint.latitude,
            )

            if distance_to_waypoint_m < closest_m:
                closest_m, index_of_closest = distance_to_waypoint_m, waypoint_index

            if distance_to_waypoint_m < cs.km_to_meters(global_path_spacing_km):
                return waypoint_index

        return index_of_closest

    def update_params(self):
        """Update instance variables that depend on parameters if they have changed."""

        mode = self.get_parameter("mode").get_parameter_value().string_value
        if mode != self.mode:
            self.get_logger().debug(f"switching from {self.mode} mode to {mode} mode")
            self.mode = mode

        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        if pub_period_sec != self.pub_period_sec:
            self.get_logger().debug(
                f"Updating pub period and timers from {self.pub_period_sec} to {pub_period_sec}"
            )
            self.pub_period_sec = pub_period_sec
            self.desired_heading_timer = self.create_timer(
                timer_period_sec=self.pub_period_sec, callback=self.desired_heading_callback
            )

        planner = self.get_parameter("path_planner").get_parameter_value().string_value
        if planner != self.planner:
            self.get_logger().debug(f"Updating planner from {self.planner} to {planner}")
            self.planner = planner

    def _all_subs_active(self) -> bool:
        return (
            self.ais_ships is not None
            and self.gps is not None
            and self.global_path is not None
            and self.filtered_wind_sensor is not None
        )

    def _has_gps_timed_out(self) -> bool:
        """Checks if we haven't received a GPS message for more than 2 minutes."""

        elapsed_sec = self._now_sec() - self.gps_timeout_start_sec
        return elapsed_sec > GPS_TIMEOUT_SEC

    def _log_inactive_subs_warning(self):
        """
        Logs a warning message for each inactive subscriber.
        """
        inactive_subs = []
        if self.ais_ships is None:
            inactive_subs.append("ais_ships")
        if self.gps is None:
            inactive_subs.append("gps")
        if self.global_path is None:
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
