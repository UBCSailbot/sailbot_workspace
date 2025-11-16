"""
Mock wind sensor node for ROS 2.

This node publishes mock data as if it were real apparent wind speed and direction (AWSD)
data coming from the wind sensors on the boat. It obtains true wind speed and direction data
(TWSD) from either a statistical model or user input and uses the current boat heading and
speed to convert the TWSD to AWSD and publishes the AWSD to the filtered_wind_sensor topic.

Node name:
- mock_wind_sensor

Publishes:
- topic: filtered_wind_sensor (custom_interfaces/WindSensor)
  - speed:  (HelperSpeed)
  - direction: degrees in boat coordinates (int), wrt boat_coordinates (refer to WindSensor.msg)

Subscribes:
- topic: gps (custom_interfaces/GPS)
  - used for current boat heading and speed

Parameters:
- pub_period_sec (double, required)
  - Publish period in seconds.
- mean_wind_speed (double, default: 16.0)
  - Mean true wind speed (kmph).
- mean_direction (int, default: 30)
  - Mean true wind direction (degrees, global frame).
- sd_speed (double, default: 1.0)
  - Standard deviation for wind speed (kmph).
- mode (string, default: "variable")
  - "variable": samples from the distributions.
  - "constant": always publishes the mean values.
- direction_kappa (double, default: 50.0)
  - Von Mises concentration for direction sampling; higher = tighter around the mean.
  - Typical values: 10–100.

Behavior:
- Wind speed is sampled from a Gamma distribution whose shape/scale are chosen to match the
  configured mean and standard deviation. This yields a positively skewed distribution.
- Direction is sampled from a von Mises distribution in radians around the mean apparent wind
  direction, then converted to degrees and into the boat frame.
- To reduce chattering, sampled values are held constant for a random duration between
  60 and 120 publish ticks before refreshing.

Example runtime usage:
- ros2 param set /mock_wind_sensor direction_kappa 75
- ros2 param set /mock_wind_sensor mode constant
- ros2 param set /mock_wind_sensor mean_wind_speed 12.5
- ros2 param set /mock_wind_sensor mean_direction 12.5
- ros2 param set /mock_wind_sensor sd_speed 3.0

All the declared parameters can be set this way.

USES constants defined in mock_nodes.shared_constants
"""

import random
import custom_interfaces.msg as ci
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.stats import gamma, vonmises

import local_pathfinding.coord_systems as cs
import local_pathfinding.mock_nodes.shared_constants as sc
import local_pathfinding.wind_coord_systems as wcs


class MockWindSensor(Node):
    """Publishes mock apparent wind speed and direction at a fixed rate.

    See the module docstring for topics and parameters.
    """

    def __init__(self):
        """Initialize timers, publishers, subscriptions, and declare parameters."""
        super().__init__("mock_wind_sensor")

        # Declare ROS parameters (qos depth and publish period)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("mean_wind_speed", 10.0),
                ("mean_direction", 30),  # from the bow to the stern of the boat
                ("sd_speed", 1.0),
                ("mode", "variable"),  # set constant for fixing the value to a constant
                ("direction_kappa", 50.0),  # concentration for von Mises (higher = tighter)
            ],
        )

        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )

        # Mock wind sensor timer
        self.__mock_wind_sensor_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_wind_sensor_callback
        )

        # Mock wind sensor publisher initialization
        self.__wind_sensors_pub = self.create_publisher(
            msg_type=ci.WindSensor,
            topic="filtered_wind_sensor",
            qos_profile=10,
        )

        self.__gps_sub = self.create_subscription(
            msg_type=ci.GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )

        self.__last_direction = int(
            self.get_parameter("mean_direction").get_parameter_value().double_value
        )

        self.__last_speed = ci.HelperSpeed(
            speed=self.get_parameter("mean_wind_speed").get_parameter_value().double_value
        )

        self.__ticks = random.randint(60, 120)
        self.__ticks_so_far = 0
        self.__start = True
        self.__boat_heading = sc.START_HEADING
        self.__boat_speed = sc.MEAN_SPEED

    def mock_wind_sensor_callback(self) -> None:
        """Timer callback to sample or hold wind values and publish a WindSensor message."""
        self.get_latest_speed_and_direction_values()

        wind_speed_, direction = self.get_speed_and_direction()

        msg = ci.WindSensor()
        msg.speed, msg.direction = wind_speed_, direction

        self.get_logger().debug(f"Publishing to {self.__wind_sensors_pub.topic}: {msg}")
        self.__wind_sensors_pub.publish(msg)

    def get_speed_and_direction(self):

        if self.__mode == "constant" or self.__start:
            return ci.HelperSpeed(speed=self.__mean_wind_speed)

        if self.__ticks_so_far < self.__ticks:
            self.__ticks_so_far += 1
            return self.__last_speed, self.__last_direction

        mean = self.__mean_wind_speed
        sd = self.__sd_wind_speed

        k = (mean / sd) ** 2
        theta = (sd**2) / mean

        wind_speed = gamma.rvs(a=k, scale=theta)
        self.__last_speed = ci.HelperSpeed(speed=abs(wind_speed))

        # Convert true wind to apparent wind (global frame)
        mean_aw_direction, mean_aw_speed = wcs.get_apparent_wind(
            self.__mean_direction, wind_speed,  self.__boat_heading, self.__boat_speed
        )



      def get_mock_wind_speed(self) -> ci.HelperSpeed:
        """Return a wind speed sample or a held value.

        - In "constant" mode, returns the configured mean speed.
        - Otherwise, holds the previous sample for 60–120 ticks. When refreshing, samples
          from a Gamma distribution parameterized to match the configured mean and SD.
        """
        if self.__mode == "constant" or self.__start:
            return ci.HelperSpeed(speed=self.__mean_wind_speed)

        if self.__ticks_so_far < self.__ticks:
            self.__ticks_so_far += 1
            return self.__last_speed

        mean = self.__mean_wind_speed
        sd = self.__sd_wind_speed

        k = (mean / sd) ** 2
        theta = (sd**2) / mean

        wind_speed_ = gamma.rvs(a=k, scale=theta)
        self.__last_speed = ci.HelperSpeed(speed=abs(wind_speed_))
        return self.__last_speed

    def get_direction_value(self) -> int:
        """Return a wind direction sample (degrees, boat frame) or a held value.

        - In "constant" mode, returns the configured mean direction (converted to boat frame).
        - Otherwise, holds the previous sample for 60–120 ticks. When refreshing:
          1) Converts mean true wind to mean apparent wind given boat heading/speed.
          2) Samples from a von Mises distribution centered at that mean with concentration
             'direction_kappa' (higher = tighter).
          3) Converts the sample to degrees and into the boat coordinate frame.
        """
        # Convert true wind to apparent wind (global frame)
        mean_aw_direction, _ = wcs.get_apparent_wind(
            self.__mean_direction, self.__mean_wind_speed, self.__boat_heading, self.__boat_speed
        )

        if self.__mode == "constant" or self.__start:
            return int(
              wcs.global_to_boat_coordinate(self.__boat_heading, cs.bound_to_180(np.degrees()))
            )

        if self.__ticks_so_far < self.__ticks:
            self.__ticks_so_far += 1
            return self.__last_direction

        self.__ticks = random.randint(60, 120)
        self.__ticks_so_far = 0


        # Sample around the mean apparent wind in radians and convert back to degrees
        mean_direction_rad = np.radians(mean_aw_direction)
        direction_rad = vonmises.rvs(kappa=self.__direction_kappa, loc=mean_direction_rad, size=1)[0] # noqa
        direction_deg = np.degrees(direction_rad)

        result = int(
            wcs.global_to_boat_coordinate(self.__boat_heading, cs.bound_to_180(direction_deg))
        )
        self.__last_direction = result
        return result

    def get_latest_speed_and_direction_values(self) -> None:
        """Updates mean wind speed and direction with the latest values from ROS parameters."""

        self.__mean_wind_speed = (
            self.get_parameter("mean_wind_speed").get_parameter_value().double_value
        )

        self.__mean_direction = (
            self.get_parameter("mean_direction").get_parameter_value().integer_value
        )

        self.__sd_wind_speed = self.get_parameter("sd_speed").get_parameter_value().double_value

        self.__mode = self.get_parameter("mode").get_parameter_value().string_value
        self.__direction_kappa = (
            self.get_parameter("direction_kappa").get_parameter_value().double_value
        )

    def gps_callback(self, msg: ci.GPS) -> None:
        """Callback function for the GPS subscription. Updates the boat's position.

        Args:
            msg (ci.GPS): The GPS message containing the boat's position.
        """

        self.get_logger().debug(f"received n {self.__wind_sensors_pub.topic}: {msg}")
        self.__start = False
        self.__boat_heading = msg.heading.heading
        self.__boat_speed = msg.speed.speed


def main(args=None):
    rclpy.init(args=args)
    mock_wind_sensor = MockWindSensor()

    rclpy.spin(node=mock_wind_sensor)

    mock_wind_sensor.destroy_node()
    rclpy.shutdown()
