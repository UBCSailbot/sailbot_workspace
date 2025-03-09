"""The main node of the local_pathfinding package, represented by the `Sailbot` class."""

import custom_interfaces.msg as ci
import rclpy
from rclpy.node import Node

from local_pathfinding.local_path import LocalPath


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
        lpath_data_pub (Publisher): Publish the local path in a `LPathData` msg.

    Publisher timers:
        pub_period_sec (float): The period of the publisher timers.
        desired_heading_timer (Timer): Call the desired heading callback function.
        lpath_data_timer (Timer): Call the local path callback function.

    Attributes from subscribers:
        ais_ships (ci.AISShips): Data from other boats.
        gps (ci.GPS): Data from the GPS sensor.
        global_path (ci.Path): Path that we are following.
        filtered_wind_sensor (ci.WindSensor): Filtered data from the wind sensors.

    Attributes:
        local_path (LocalPath): The path that `Sailbot` is following.
        planner (str): The path planner that `Sailbot` is using.
    """

    def __init__(self):
        super().__init__(node_name="navigate")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("path_planner", rclpy.Parameter.Type.STRING),
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
        self.lpath_data_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.lpath_data_callback
        )

        # attributes from subscribers
        self.ais_ships = None
        self.gps = None
        self.global_path = None
        self.filtered_wind_sensor = None

        # attributes
        self.local_path = LocalPath(parent_logger=self.get_logger())
        self.planner = self.get_parameter("path_planner").get_parameter_value().string_value
        self.get_logger().debug(f"Got parameter: {self.planner=}")

    # subscriber callbacks

    def ais_ships_callback(self, msg: ci.AISShips):
        self.get_logger().debug(f"Received data from {self.ais_ships_sub.topic}: {msg}")
        self.ais_ships = msg

    def gps_callback(self, msg: ci.GPS):
        self.get_logger().debug(f"Received data from {self.gps_sub.topic}: {msg}")
        self.gps = msg

    def global_path_callback(self, msg: ci.Path):
        self.get_logger().debug(f"Received data from {self.global_path_sub.topic}: {msg}")
        self.global_path = msg

    def filtered_wind_sensor_callback(self, msg: ci.WindSensor):
        self.get_logger().debug(f"Received data from {self.filtered_wind_sensor_sub.topic}: {msg}")
        self.filtered_wind_sensor = msg

    # publisher callbacks

    def desired_heading_callback(self):
        """Get and publish the desired heading.

        Warn if not following the heading conventions in custom_interfaces/msg/HelperHeading.msg.
        """
        self.update_params()

        desired_heading = self.get_desired_heading()
        if desired_heading < 0 or 360 <= desired_heading:
            self.get_logger().warning(f"Heading {desired_heading} not in [0, 360)")

        msg = ci.DesiredHeading()
        msg.heading.heading = desired_heading

        self.desired_heading_pub.publish(msg)
        self.get_logger().debug(f"Publishing to {self.desired_heading_pub.topic}: {msg}")

    def lpath_data_callback(self):
        """Get and publish the local path."""
        if self.local_path.waypoints is None:
            current_local_path = ci.Path(waypoints=list())
        else:
            current_local_path = ci.Path(waypoints=self.local_path.waypoints)

        msg = ci.LPathData(local_path=current_local_path)

        self.lpath_data_pub.publish(msg)
        self.get_logger().debug(f"Publishing to {self.lpath_data_pub.topic}: {msg}")

    # helper functions

    def get_desired_heading(self) -> float:
        """Get the desired heading.

        Returns:
            float: The desired heading if all subscribers are active, else a number that violates
                the heading convention.
        """
        if not self._all_subs_active():
            self._log_inactive_subs_warning()
            return -1.0

        self.local_path.update_if_needed(
            self.gps, self.ais_ships, self.global_path, self.filtered_wind_sensor, self.planner
        )

        # TODO: create function to compute the heading from current position to next local waypoint
        return 0.0

    def update_params(self):
        """Update instance variables that depend on parameters if they have changed."""
        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        if pub_period_sec != self.pub_period_sec:
            self.get_logger().debug(
                f"Updating pub period and timers from {self.pub_period_sec} to {pub_period_sec}"
            )
            self.pub_period_sec = pub_period_sec
            self.desired_heading_timer = self.create_timer(
                timer_period_sec=self.pub_period_sec, callback=self.desired_heading_callback
            )
            self.lpath_data_timer = self.create_timer(
                timer_period_sec=self.pub_period_sec, callback=self.lpath_data_callback
            )

        planner = self.get_parameter("path_planner").get_parameter_value().string_value
        if planner != self.planner:
            self.get_logger().debug(f"Updating planner from {self.planner} to {planner}")
            self.planner = planner

    def _all_subs_active(self) -> bool:
        return True  # TODO: this line is a placeholder, delete when mocks can be run
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
