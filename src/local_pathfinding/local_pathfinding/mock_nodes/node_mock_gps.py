"""
Mock class for the GPS. Publishes basic GPS data to the ROS network.

USES constants defined in mock_nodes.shared_utils
"""

import math
from typing import List

import custom_interfaces.msg as ci
import rclpy
from geopy.distance import great_circle
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

import local_pathfinding.coord_systems as cs
import local_pathfinding.mock_nodes.shared_utils as sc
from local_pathfinding.ompl_objectives import TimeObjective
from test_plans.test_plan import TestPlan

SECONDS_PER_HOUR = 3600


class MockGPS(Node):

    def __init__(self) -> None:
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

        # Type annotations for instance variables
        self.__current_location: ci.HelperLatLon
        self.__mean_speed_kmph: ci.HelperSpeed
        self.__heading_deg: ci.HelperHeading
        self.__tw_dir_deg: int
        self.__tw_speed_kmph: float

        # Declare ROS parameters (qos depth and publish period).
        # tw_speed_kmph and tw_dir_deg must be loaded via wind_params.sh script.
        # Do NOT use ros2 param set for these values as it causes mismatch with mock_wind_sensor.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("test_plan", rclpy.Parameter.Type.STRING),
            ],
        )

        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )
        self.test_plan = self.get_parameter("test_plan").get_parameter_value().string_value

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

        # Read attributes from test_plan and update attributes
        self.test_plan = self.get_parameter("test_plan").get_parameter_value().string_value

        self.initialize_sailbot_state()

        # Mock GPS timer
        self.__mock_gps_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_gps_callback
        )

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

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        """ROS2 parameter update callback.

        Applies updates to true wind speed/direction. Values take effect on the next publish tick.
        """
        try:
            for p in params:
                if p.name == "tw_dir_deg":
                    new_direction_deg = int(p.value)
                    sc.validate_tw_dir_deg(new_direction_deg)
                    self.__tw_dir_deg = new_direction_deg
                else:
                    self.__tw_speed_kmph = p.value
            return SetParametersResult(successful=True)
        except Exception:
            reason = "Please enter the direction in (-180, 180]."
            return SetParametersResult(successful=False, reason=reason)

    def update_speed(self):
        """Update the boat speed based on current heading and true wind.

        Uses TimeObjective.get_sailbot_speed to calculate realistic boat speed
        given the current heading relative to true wind direction and speed.
        """
        self.__mean_speed_kmph = ci.HelperSpeed(
            speed=float(
                TimeObjective.get_sailbot_speed(
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

    def desired_heading_callback(self, msg: ci.DesiredHeading) -> None:
        """Callback for topic desired heading

        Args:
            msg (ci.DesiredHeading): The desired heading for the boat.
        """
        self._logger.debug(f"Received data from {self.__desired_heading_sub.topic}: {msg}")
        self.__heading_deg = msg.heading

    def initialize_sailbot_state(self) -> None:
        """Initialize the sailbot state from the test plan file."""

        test_plan = TestPlan(file_name=self.test_plan)

        # Load sailbot state
        self.__mean_speed_kmph, self.__heading_deg, self.__current_location = (
            test_plan.sailbot_state
        )

        # Load wind data
        self.__tw_speed_kmph, self.__tw_dir_deg = test_plan.wind_data


def main(args=None):
    rclpy.init(args=args)
    mock_gps = MockGPS()

    rclpy.spin(node=mock_gps)

    mock_gps.destroy_node()
    rclpy.shutdown()
