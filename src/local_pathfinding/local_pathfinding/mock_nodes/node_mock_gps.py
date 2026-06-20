"""
Mock class for the GPS. Publishes basic GPS data to the ROS network.
"""

import math
import random
from typing import List

import rclpy
from geopy.distance import great_circle
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs
from local_pathfinding.ompl_objectives import TimeObjective

SECONDS_PER_HOUR = 3600
GPS_NOISE_SIGMA_METERS = 0.5
DRIFT_SPEED_NOISE_SIGMA_SCALE = 0.05
DRIFT_DIR_NOISE_SIGMA_DEG = 2.0


class MockGPS(Node):

    def __init__(self) -> None:
        """Initialize the MockGPS class. The class is used to publish mock gps
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
                ("use_gps_noise", rclpy.Parameter.Type.BOOL),
                ("use_ocean_drift", rclpy.Parameter.Type.BOOL),
                ("use_drift_randomization", rclpy.Parameter.Type.BOOL),
                ("ocean_drift_speed_kmph", rclpy.Parameter.Type.DOUBLE),
                ("ocean_drift_dir_deg", rclpy.Parameter.Type.DOUBLE),
                ("ocean_drift_accel_kmph2", rclpy.Parameter.Type.DOUBLE),
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

        self._use_noise = self.get_parameter("use_gps_noise").get_parameter_value().bool_value
        self._use_drift = self.get_parameter("use_ocean_drift").get_parameter_value().bool_value
        self._use_drift_randomization = (
            self.get_parameter("use_drift_randomization").get_parameter_value().bool_value
        )
        self._drift_speed_kmph = (
            self.get_parameter("ocean_drift_speed_kmph").get_parameter_value().double_value
        )
        self._drift_dir_deg = (
            self.get_parameter("ocean_drift_dir_deg").get_parameter_value().double_value
        )
        self._drift_accel_kmph2 = (
            self.get_parameter("ocean_drift_accel_kmph2").get_parameter_value().double_value
        )
        self._drift_offset_km = cs.XY(0.0, 0.0)  # cumulative offset in km

        self.get_logger().debug(
            f"ROS2 parameters: use_gps_noise={self._use_noise}, "
            f"use_ocean_drift={self._use_drift}, "
            f"use_drift_randomization={self._use_drift_randomization}, "
            f"ocean_drift_speed_kmph={self._drift_speed_kmph}, "
            f"ocean_drift_dir_deg={self._drift_dir_deg}, "
            f"ocean_drift_accel_kmph2={self._drift_accel_kmph2}, "
            f"pub_period_sec={self.pub_period_sec}"
        )

        # Mock GPS publisher initialization
        self._gps_pub = self.create_publisher(
            msg_type=ci.GPS,
            topic="gps",
            qos_profile=10,
        )

        # Desired heading subscriber
        self._desired_heading_sub = self.create_subscription(
            msg_type=ci.DesiredHeading,
            topic="desired_heading",
            callback=self.desired_heading_callback,
            qos_profile=10,
        )

        # Mock GPS timer
        self._mock_gps_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_gps_callback
        )

        # Parameter Event Handler (Parameters can change over the life of the simulation)
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def add_gps_noise(self, lat_lon_msg: ci.HelperLatLon) -> ci.HelperLatLon:
        """Adds Gaussian noise to a coordinate.

        Args:
            lat_lon_msg (ci.HelperLatLon): The original lat/lon coordinates.

        Returns:
            ci.HelperLatLon: Object containing the new lat/lon coordinates with noise.
        """
        noise_km = cs.XY(
            random.gauss(0.0, GPS_NOISE_SIGMA_METERS) / 1000,
            random.gauss(0.0, GPS_NOISE_SIGMA_METERS) / 1000,
        )
        return cs.xy_to_latlon(reference=lat_lon_msg, xy=noise_km)

    def add_ocean_drift(self, lat_long_msg: ci.HelperLatLon) -> ci.HelperLatLon:
        """Applies a current-based offset to the GPS position.


        Args:
            lat_long_msg (ci.HelperLatLon): The original lat/lon coordinates.

        Returns:
            ci.HelperLatLon: Object containing the new lat/lon coordinates with ocean drift.
        """

        dt_hours = self.pub_period_sec / SECONDS_PER_HOUR

        self._drift_speed_kmph += self._drift_accel_kmph2 * dt_hours

        drift_speed_kmph = self._drift_speed_kmph
        drift_dir_deg = self._drift_dir_deg

        if self._use_drift_randomization:
            drift_speed_kmph += random.gauss(0.0, drift_speed_kmph * DRIFT_SPEED_NOISE_SIGMA_SCALE)
            drift_dir_deg += random.gauss(0.0, DRIFT_DIR_NOISE_SIGMA_DEG)

        drift_dir_rad = math.radians(drift_dir_deg)
        step_km = drift_speed_kmph * dt_hours
        self._drift_offset_km = cs.XY(
            self._drift_offset_km.x + step_km * math.sin(drift_dir_rad),
            self._drift_offset_km.y + step_km * math.cos(drift_dir_rad),
        )
        return cs.xy_to_latlon(reference=lat_long_msg, xy=self._drift_offset_km)

    def mock_gps_callback(self) -> None:
        """Updates boat speed based on current heading and true wind.
        Publishes mock gps data.
        """
        self.update_speed()
        self.get_next_location()

        self.get_logger().info(
            f"Actual Lat: {self._current_location.latitude:.7f} "
            f"Actual Lon: {self._current_location.longitude:.7f}\n"
        )

        if self._use_drift:
            published_location = self.add_ocean_drift(self._current_location)

            self.get_logger().info(
                f"Drift Offset: {self._drift_offset_km}\n"
                f"Drift Speed: {self._drift_speed_kmph}\n"
                f"Drift Direction {self._drift_dir_deg}"
            )
        else:
            published_location = self._current_location

        if self._use_noise:
            published_location = self.add_gps_noise(published_location)

            self.get_logger().info(
                f"Published Lat: {published_location.latitude:.7f} "
                f"Published Lon: {published_location.longitude:.7f} "
            )

        msg: ci.GPS = ci.GPS(
            lat_lon=published_location,
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
            elif p.name == "use_gps_noise":
                self._use_noise = bool(p.value)
            elif p.name == "use_ocean_drift":
                self._use_drift = bool(p.value)
                if not self._use_drift:
                    self._drift_offset_km = cs.XY(
                        0.0, 0.0
                    )  # resets the drift offset after it's turned off
            elif p.name == "use_drift_randomization":
                self._use_drift_randomization = bool(p.value)
            elif p.name == "ocean_drift_speed_kmph":
                self._drift_speed_kmph = float(p.value)
            elif p.name == "ocean_drift_dir_deg":
                self._drift_dir_deg = float(p.value)
            elif p.name == "ocean_drift_accel_kmph2":
                self._drift_accel_kmph2 = float(p.value)
            else:
                self._tw_speed_kmph = p.value
        return SetParametersResult(successful=True)

    def update_speed(self) -> None:
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
