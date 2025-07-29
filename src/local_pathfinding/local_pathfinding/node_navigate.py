"""The main node of the local_pathfinding package, represented by the `Sailbot` class."""

import json
import os

import custom_interfaces.msg as ci
import rclpy
from pyproj import Geod
from rclpy.node import Node
from shapely.geometry import MultiPolygon, Polygon

import local_pathfinding.coord_systems as cs
import local_pathfinding.global_path as gp
import local_pathfinding.obstacles as ob
from local_pathfinding.local_path import LocalPath

WAYPOINT_REACHED_THRESH_KM = 0.5
GEODESIC = Geod(ellps="WGS84")


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

    Attributes:
        local_path (LocalPath): The path that `Sailbot` is following.
        planner (str): The path planner that `Sailbot` is using.
    """

    def __init__(self):
        super().__init__(node_name="navigate")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("mode", rclpy.Parameter.Type.STRING),
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("path_planner", rclpy.Parameter.Type.STRING),
                ("use_mock_land", False),
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
        self.desired_heading_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.desired_heading_callback
        )

        # attributes from subscribers
        self.ais_ships = None
        self.gps = None
        self.global_path = None
        self.filtered_wind_sensor = None
        self.desired_heading = None

        # attributes
        self.local_path = LocalPath(parent_logger=self.get_logger())
        self.current_waypoint_index = 0
        self.use_mock_land = self.get_parameter("use_mock_land").get_parameter_value().bool_value
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.planner = self.get_parameter("path_planner").get_parameter_value().string_value
        self.get_logger().debug(f"Got parameter: {self.planner=}")

        # Initialize mock land obstacle
        self.land_multi_polygon = None
        if self.use_mock_land:

            # find the mock land file
            current_dir = os.path.dirname(os.path.abspath(__file__))
            mock_land_dir = os.path.join(current_dir, "..", "land", "mock")
            self.mock_land_file = os.path.join(mock_land_dir, "mock_land.json")

            with open(self.mock_land_file, "r") as f:
                data = json.load(f)
                polygons = [Polygon(p) for p in data.get("land_polygons", [])]
                self.land_multi_polygon = MultiPolygon(polygons)
                self.get_logger().info("Loaded mock land data.")

        # Simulated voyage metrics
        self._desired_heading_change_count = 0
        self._desired_heading_callback_cycles = 0
        self._path_switch_count = 0

    # subscriber callbacks
    def ais_ships_callback(self, msg: ci.AISShips):
        self.get_logger().debug(f"Received data from {self.ais_ships_sub.topic}: {msg}")
        self.ais_ships = msg

    def gps_callback(self, msg: ci.GPS):
        self.get_logger().debug(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg

    def global_path_callback(self, msg: ci.Path):
        self.get_logger().debug(
            f"Received data from {self.global_path_sub.topic}: {gp.path_to_dict(msg)}"
        )
        self.global_path = msg

    def filtered_wind_sensor_callback(self, msg: ci.WindSensor):
        self.get_logger().debug(f"Received data from {self.filtered_wind_sensor_sub.topic}: {msg}")
        self.filtered_wind_sensor = msg

    # publisher callbacks
    def desired_heading_callback(self):
        """Get and publish the desired heading."""

        # Count this cycle
        self._desired_heading_callback_cycles += 1

        if not self._all_subs_active():
            self._log_inactive_subs_warning()
            return  # should not continue, return and try again next loop

        self.update_params()

        desired_heading = self.get_desired_heading()
        msg = ci.DesiredHeading()
        msg.heading.heading = desired_heading

        # Check if desired heading actually changed
        if self.desired_heading is None or desired_heading != self.desired_heading.heading.heading:
            self.desired_heading_change_count += 1
            self.get_logger().info(f"Updating desired heading to: {msg.heading.heading:.2f}")

        self.desired_heading = msg

        # log the ratio every 10 cycles
        if self.desired_heading_callback_cycles % 10 == 0:
            heading_ratio = self._desired_heading_change_count / self._desired_heading_callback_cycles
            switch_ratio = self._path_switch_count / self._desired_heading_callback_cycles
            self.get_logger().debug(
                f"Desired heading changes to cycles ratio: {heading_ratio:.2f}"
                f"Path switch ratio: {switch_ratio:.2f}"
            )

        self.get_logger().debug(
            f"Publishing to {self.desired_heading_pub.topic}: {msg.heading.heading}"
        )
        self.desired_heading_pub.publish(msg)

        self.get_logger().debug(f"Publishing local path data to {self.lpath_data_pub.topic}")
        self.publish_local_path_data()

    def publish_local_path_data(self):
        """
        Collect all navigation data and publish it in one message.
        In development mode, all navigation data is published.
        In production mode, only the local path is published, with all other data set to 0 or empty

        """

        # publish all navigation data when in dev mode
        if self.mode == "development":
            helper_obstacles = []

            for obst in self.local_path.state.obstacles:

                if isinstance(obst, ob.Land):
                    for polygon in obst.collision_zone.geoms:
                        latlon_polygon = cs.xy_polygon_to_latlon_polygon(
                            self.local_path.state.reference_latlon, polygon
                        )

                        # each point of the polygon is in lat lon now
                        # but you cant construct a shapely polgyon out of HelperLatLon objects
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
                local_path=self.local_path.path,
                gps=self.gps,
                filtered_wind_sensor=self.filtered_wind_sensor,
                ais_ships=self.ais_ships,
                obstacles=helper_obstacles,
                desired_heading=self.desired_heading,
            )
        else:
            # in production only publish the local path for website
            msg = ci.LPathData(local_path=self.local_path.path)

        self.lpath_data_pub.publish(msg)

    # helper functions
    def get_desired_heading(self) -> float:
        """Get the desired heading.

        Returns:
            float: The desired heading
        """

        received_new_path = self.local_path.update_if_needed(
            self.gps,
            self.ais_ships,
            self.global_path,
            self.filtered_wind_sensor,
            self.planner,
            self.land_multi_polygon,
        )

        if received_new_path:
            self.current_waypoint_index = 0
            self._path_switch_count += 1

        desired_heading, self.current_waypoint_index = (
            self.calculate_desired_heading_and_waypoint_index(
                self.local_path.path, self.current_waypoint_index, self.gps.lat_lon
            )
        )

        return desired_heading

    @staticmethod
    def calculate_desired_heading_and_waypoint_index(
        path: ci.Path, waypoint_index: int, boat_lat_lon: ci.HelperLatLon
    ):
        waypoint = path.waypoints[waypoint_index]
        desired_heading, _, distance_to_waypoint_m = GEODESIC.inv(
            boat_lat_lon.longitude, boat_lat_lon.latitude, waypoint.longitude, waypoint.latitude
        )

        if cs.meters_to_km(distance_to_waypoint_m) < WAYPOINT_REACHED_THRESH_KM:
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
        return self.ais_ships and self.gps and self.global_path and self.filtered_wind_sensor

    def _log_inactive_subs_warning(self):
        """
        Logs a warning message for each inactive subscriber.
        """
        inactive_subs = []
        if self.ais_ships_sub is None:
            inactive_subs.append("ais_ships")
        if self.gps_sub is None:
            inactive_subs.append("gps")
        if self.global_path_sub is None:
            inactive_subs.append("global_path")
        if self.filtered_wind_sensor_sub is None:
            inactive_subs.append("filtered_wind_sensor")
        if len(inactive_subs) == 0:
            return
        self._logger.warning("Inactive Subscribers: " + ", ".join(inactive_subs))


if __name__ == "__main__":
    main()
