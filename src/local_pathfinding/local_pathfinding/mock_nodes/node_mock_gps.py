"""
Mock class for the GPS. Publishes basic GPS data to the ROS network.

USES constants defined in mock_nodes.shared_constants
"""

import math

import custom_interfaces.msg as ci
import rclpy
from geopy.distance import great_circle
from rclpy.node import Node

import local_pathfinding.coord_systems as cs
import local_pathfinding.mock_nodes.shared_utils as sc
from local_pathfinding.ompl_objectives import TimeObjective

SECONDS_PER_HOUR = 3600


class MockGPS(Node):

    def __init__(self):
        """Initialize the MockGPS class. The class is used to publish mock mock gps
        data to the ROS network.

        Args:
            Node (Node): The ROS node that the class will run on.

        Attributes:
            __mock_gps_timer (Timer): Timer to call the mock gps callback function.
            __mock_gps_publisher (Publisher): Publisher for the gps data.
            __mean_speed_kmph (HelperSpeed): Constant speed of the boat.
            __current_location (HelperLatLon): Current location of the boat.
            __mean_heading (HelperHeading): Constant heading of the boat.
        """
        super().__init__("mock_gps")

        # Declare ROS parameters (qos depth and publish period).
        # tw_speed_kmph and tw_dir_deg must be loaded via wind_params.sh script.
        # Do NOT use ros2 param set for these values as it causes mismatch with mock_wind_sensor.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("tw_speed_kmph", sc.TW_SPEED_KMPH),
                ("tw_dir_deg", sc.TW_DIRECTION_DEG),
            ],
        )

        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )

        # Mock GPS timer
        self.__mock_gps_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_gps_callback
        )

        # Mock GPS publisher initialization
        self.__gps_pub = self.create_publisher(
            msg_type=ci.GPS,
            topic="gps",
            qos_profile=10,
        )

        # Desired heading subscriber
        self.__desired_heading_sub = self.create_subscription(
            msg_type=ci.DesiredHeading,
            topic="desired_heading",
            callback=self.desired_heading_callback,
            qos_profile=10,
        )

        self.__mean_speed_kmph = sc.MEAN_SPEED
        self.__current_location = sc.START_POINT
        self.__heading_deg = sc.START_HEADING

        # Store true wind parameters
        self.__tw_speed_kmph = (
            self.get_parameter("tw_speed_kmph").get_parameter_value().double_value
        )
        self.__tw_dir_deg = self.get_parameter("tw_dir_deg").get_parameter_value().integer_value

    def mock_gps_callback(self) -> None:
        """Callback function for the mock GPS timer. Publishes mock gps data to the ROS
        network.
        """
        # Update boat speed based on current heading and true wind
        self.update_speed()
        self.get_next_location()

        msg: ci.GPS = ci.GPS(
            lat_lon=self.__current_location,
            speed=self.__mean_speed_kmph,
            heading=self.__heading_deg,
        )
        self.get_logger().debug(f"Publishing to {self.__gps_pub.topic}, heading: {msg.heading}")
        self.get_logger().debug(f"Publishing to {self.__gps_pub.topic}, speed: {msg.speed}")
        self.get_logger().debug(
            f"Publishing to {self.__gps_pub.topic}, latitude: {msg.lat_lon.latitude}"
        )
        self.get_logger().debug(
            f"Publishing to {self.__gps_pub.topic}, longitude: {msg.lat_lon.longitude}"
        )
        self.__gps_pub.publish(msg)

    def update_speed(self):
        """Update the boat speed based on current heading and true wind.

        Uses TimeObjective.get_sailbot_speed to calculate realistic boat speed
        given the current heading relative to true wind direction and speed.
        """
        self.__mean_speed_kmph = ci.HelperSpeed(
            speed=float(
                100.0 * TimeObjective.get_sailbot_speed(
                    math.radians(self.__heading_deg.heading),
                    math.radians(self.__tw_dir_deg),
                    self.__tw_speed_kmph,
                )
            )
        )
        self.get_logger().debug(
            f"Updated speed: {self.__mean_speed_kmph.speed:.2f} kmph "
            f"(heading: {self.__heading_deg.heading:.1f}°, "
            f"TW: {self.__tw_speed_kmph:.1f} kmph from {self.__tw_dir_deg}°)"
        )

    def get_next_location(self) -> None:
        """Get the next location by following the great circle. Assumes constant speed and heading"""  # noqa
        # distance travelled = speed * callback time (s)
        distance_km = self.__mean_speed_kmph.speed * (self.pub_period_sec / SECONDS_PER_HOUR)
        start = (
            self.__current_location.latitude,
            self.__current_location.longitude,
        )
        heading_degrees = self.__heading_deg.heading
        destination = great_circle(kilometers=distance_km).destination(start, heading_degrees)
        self.__current_location = ci.HelperLatLon(
            latitude=destination.latitude, longitude=destination.longitude
        )
        self._logger.debug(
            f"Distance Travelled: {cs.km_to_meters(distance_km):.2f} m, direction: {self.__heading_deg.heading:.1f} deg,  speed: {self.__mean_speed_kmph.speed:.2f} kmph"  # noqa
        )

    def desired_heading_callback(self, msg: ci.DesiredHeading):
        """Callback for topic desired heading

        Args:
            msg (ci.DesiredHeading): The desired heading for the boat.
        """
        self._logger.debug(f"Received data from {self.__desired_heading_sub.topic}: {msg}")
        self.__heading_deg = msg.heading


def main(args=None):
    rclpy.init(args=args)
    mock_gps = MockGPS()

    rclpy.spin(node=mock_gps)

    mock_gps.destroy_node()
    rclpy.shutdown()
