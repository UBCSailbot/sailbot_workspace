"""
Mock class for the GPS. Publishes basic GPS data to the ROS network.
"""

import math
from typing import List

import custom_interfaces.msg as ci
import rclpy
from geopy.distance import great_circle
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from test_plan import TestPlan

import local_pathfinding.coord_systems as cs
from local_pathfinding.ompl_objectives import TimeObjective

SECONDS_PER_HOUR = 3600


class MockGPS(Node):

    def __init__(self) -> None:
        """Initialize the MockGPS class. The class is used to publish mock mock gps
        data to the ROS network.

        Args:
            Node (Node): The ROS node that the class will run on.

        Attributes:
            _mock_gps_timer (Timer): Timer to call the mock gps callback function.
            _mock_gps_publisher (Publisher): Publisher for the gps data.
            _mean_speed_kmph (HelperSpeed): Constant speed of the boat.
            _current_location (HelperLatLon): Current location of the boat.
            _mean_heading (HelperHeading): Constant heading of the boat.
        """
        super().__init__("mock_gps")

        # tw_speed_kmph and tw_dir_deg must be loaded via wind_params.sh script.
        # Do NOT use ros2 param set for these values as it causes mismatch with mock_wind_sensor.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("tw_speed_kmph", rclpy.Parameter.Type.DOUBLE),
                ("tw_dir_deg", rclpy.Parameter.Type.INTEGER),
                ("test_plan", rclpy.Parameter.Type.STRING),
            ],
        )

        test_plan = TestPlan(self.get_parameter("test_plan").get_parameter_value().string_value)
        gps_data = test_plan.gps

        self._tw_speed_kmph = test_plan.tw_speed_kmph
        self._tw_dir_deg = test_plan.tw_dir_deg
        self._mean_speed_kmph = ci.HelperSpeed(speed=gps_data.speed.speed)
        self._heading_deg = ci.HelperHeading(heading=gps_data.heading.heading)
        self._current_location = ci.HelperLatLon(
            latitude=gps_data.lat_lon.latitude, longitude=gps_data.lat_lon.longitude
        )

        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )

        self._gps_pub = self.create_publisher(
            msg_type=ci.GPS,
            topic="gps",
            qos_profile=10,
        )

        self._desired_heading_sub = self.create_subscription(
            msg_type=ci.DesiredHeading,
            topic="desired_heading",
            callback=self.desired_heading_callback,
            qos_profile=10,
        )

        self._mock_gps_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_gps_callback
        )

    def mock_gps_callback(self) -> None:
        """Updates boat speed based on current heading and true wind.
        Publishes mock gps data.
        """
        self.update_speed()
        self.get_next_location()

        msg: ci.GPS = ci.GPS(
            lat_lon=self._current_location,
            speed=self._mean_speed_kmph,
            heading=self._heading_deg,
        )
        self.get_logger().debug(f"Publishing to {self._gps_pub.topic}, heading: {msg.heading}")
        self.get_logger().debug(f"Publishing to {self._gps_pub.topic}, speed: {msg.speed}")
        self.get_logger().debug(
            f"Publishing to {self._gps_pub.topic}, latitude: {msg.lat_lon.latitude}"
        )
        self.get_logger().debug(
            f"Publishing to {self._gps_pub.topic}, longitude: {msg.lat_lon.longitude}"
        )
        self._gps_pub.publish(msg)

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        """This callback function serves as a guard to ensure values entered with `ros2 param set`
        are valid before they are assigned to the parameters.

        Applies updates to true wind speed/direction. Values take effect on the next publish tick.

        Rejects if tw_dir_deg is not in (-180, 180].
        """
        for p in params:
            if p.name == "tw_dir_deg":
                tw_dir_deg = int(p.value)
                if tw_dir_deg <= -180 or tw_dir_deg > 180:
                    return SetParametersResult(
                        successful=False, reason="tw_dir_deg must be in (-180, 180]"
                    )
                self._tw_dir_deg = tw_dir_deg
            else:
                self._tw_speed_kmph = p.value
        return SetParametersResult(successful=True)

    def update_speed(self):
        """Update the boat speed based on current heading and true wind.

        Uses TimeObjective.get_sailbot_speed to calculate realistic boat speed
        given the current heading relative to true wind direction and speed.
        """
        self._mean_speed_kmph = ci.HelperSpeed(
            speed=float(
                TimeObjective.get_sailbot_speed(
                    math.radians(self._heading_deg.heading),
                    math.radians(self._tw_dir_deg),
                    self._tw_speed_kmph,
                )
            )
        )
        self.get_logger().debug(
            f"Updated speed: {self._mean_speed_kmph.speed:.2f} kmph "
            f"(heading: {self._heading_deg.heading:.1f}°, "
            f"TW: {self._tw_speed_kmph:.1f} kmph from {self._tw_dir_deg}°)"
        )

    def get_next_location(self) -> None:
        """Get the next location by following the great circle. Assumes constant speed and heading"""  # noqa
        # distance travelled = speed * callback time (s)
        distance_km = self._mean_speed_kmph.speed * (self.pub_period_sec / SECONDS_PER_HOUR)
        start = (
            self._current_location.latitude,
            self._current_location.longitude,
        )
        heading_degrees = self._heading_deg.heading
        destination = great_circle(kilometers=distance_km).destination(start, heading_degrees)
        self._current_location = ci.HelperLatLon(
            latitude=destination.latitude, longitude=destination.longitude
        )
        self._logger.debug(
            f"Distance Travelled: {cs.km_to_meters(distance_km):.2f} m, direction: {self._heading_deg.heading:.1f} deg,  speed: {self._mean_speed_kmph.speed:.2f} kmph"  # noqa
        )

    def desired_heading_callback(self, msg: ci.DesiredHeading) -> None:
        """Callback for topic desired heading

        Args:
            msg (ci.DesiredHeading): The desired heading for the boat.
        """
        self._logger.debug(f"Received data from {self._desired_heading_sub.topic}: {msg}")
        self._heading_deg = msg.heading


def main(args=None):
    rclpy.init(args=args)
    mock_gps = MockGPS()

    rclpy.spin(node=mock_gps)

    mock_gps.destroy_node()
    rclpy.shutdown()
