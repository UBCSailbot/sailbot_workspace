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
- mean_direction_deg (int, default: 30)
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
- ros2 param set /mock_wind_sensor mean_direction_deg 12.5
- ros2 param set /mock_wind_sensor sd_speed 3.0

All the declared parameters can be set this way.

USES constants defined in mock_nodes.shared_constants
"""

import custom_interfaces.msg as ci
import numpy as np
import rclpy
from rclpy.node import Node
from abc import ABC, abstractmethod
from typing import Tuple
import local_pathfinding.mock_nodes.shared_constants as sc
import local_pathfinding.wind_coord_systems as wcs


class WindModel(ABC):
    @abstractmethod
    def get_wind_sensor_data(self):
        pass


class ConstantWindModel(WindModel):

    def __init__(self, true_wind_heading_deg, true_wind_speed_kmph,
                 boat_heading, boat_speed):

        self.__true_wind_heading_deg = true_wind_heading_deg
        self.__true_wind_speed_kmph = true_wind_speed_kmph
        self.__boat_heading = boat_heading
        self.__boat_speed = boat_speed

    def update(self, true_wind_heading_deg, true_wind_speed_kmph, boat_heading, boat_speed):
        self.__true_wind_heading_deg = true_wind_heading_deg
        self.__true_wind_speed_kmph = true_wind_speed_kmph
        self.__boat_heading = boat_heading
        self.__boat_speed = boat_speed

    def get_wind_sensor_data(self) -> Tuple[float, int]:
        aw_speed_kmph, aw_direction_rad = wcs.get_apparent_wind(
            self.__true_wind_heading_deg, self.__true_wind_speed_kmph, self.__boat_heading, self.__boat_speed # noqa
        )

        aw_direction_deg_boat_coord = wcs.global_to_boat_coordinate(self.__boat_heading, np.degrees(aw_direction_rad)) #noqa
        return aw_speed_kmph, int(aw_direction_deg_boat_coord)


# class VariableWindModel(WindModel):
# TODO:
#     # To be implemented
#     def __init__(self):
#         pass

#     def get_wind_sensor_data(self):
#         pass


class MockWindSensor(Node):
    def __init__(self):
        super().__init__("mock_wind_sensor")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("true_wind_speed_kmph", 10.0),
                ("true_wind_direction_deg", 0),  # from the bow to the stern of the boat
                ("sd_speed_kmph", 1.0),
                ("mode", "constant"),  # set constant for fixing the value to a constant
                ("direction_kappa", 50.0),  # concentration for von Mises (higher = tighter)
            ],
        )

        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )

        self.__mock_wind_sensor_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_wind_sensor_callback
        )
        self.__wind_sensors_pub = self.create_publisher(
            msg_type=ci.WindSensor,
            topic="filtered_wind_sensor",
            qos_profile=10,
        )
        self.__gps_sub = self.create_subscription(
            msg_type=ci.GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )
        self.__boat_heading = sc.START_HEADING.heading
        self.__boat_speed = sc.MEAN_SPEED.speed
        self.__mode = "constant"
        self.init_model()

    def init_model(self):
        self.__true_wind_speed_kmph = (
            self.get_parameter("true_wind_speed_kmph").get_parameter_value().double_value
        )

        self.__mean_direction_deg = (
            self.get_parameter("true_wind_direction_deg").get_parameter_value().integer_value
        )
        self.change_model()

    def change_model(self):
        if self.__mode == "constant":
            self.__model = ConstantWindModel(
                self.__mean_direction_deg,
                self.__true_wind_speed_kmph,
                self.__boat_heading,
                self.__boat_speed
            )
        else:
            raise NotImplementedError
        # TODO
        # self.__model = VariableWindModel()

    def update_model(self):
        if self.__mode == "constant":
            self.__model.update(
                self.__mean_direction_deg,
                self.__true_wind_speed_kmph,
                self.__boat_heading,
                self.__boat_speed
            )
        else:
            raise NotImplementedError
        # TODO
        # self.__model = VariableWindModel()

    def mock_wind_sensor_callback(self):
        self.get_latest_speed_and_direction_values()
        if self.__received_new_mode:
            self.change_model()
        else:
            self.update_model()
        aw_speed_kmph, aw_direction_boat_coord_deg = self.__model.get_wind_sensor_data()
        msg = ci.WindSensor(
            speed=ci.HelperSpeed(speed=aw_speed_kmph),
            direction=aw_direction_boat_coord_deg
        )
        self.get_logger().debug(f"Publishing to {self.__wind_sensors_pub.topic}: {msg}")
        self.__wind_sensors_pub.publish(msg)

    def get_latest_speed_and_direction_values(self) -> None:
        """Updates mean wind speed and direction with the latest values from ROS parameters."""

        self.__true_wind_speed_kmph = (
            self.get_parameter("true_wind_speed_kmph").get_parameter_value().double_value
        )

        self.__mean_direction_deg = (
            self.get_parameter("true_wind_direction_deg").get_parameter_value().integer_value
        )

        self.__sd_wind_speed = self.get_parameter("sd_speed_kmph").get_parameter_value().double_value # noqa

        self.__direction_kappa = (
            self.get_parameter("direction_kappa").get_parameter_value().double_value
        )

        mode = str(self.__mode)
        self.__mode = self.get_parameter("mode").get_parameter_value().string_value
        self.__received_new_mode = mode != self.__mode

    def gps_callback(self, msg: ci.GPS) -> None:
        """Callback function for the GPS subscription. Updates the boat's position.

        Args:
            msg (ci.GPS): The GPS message containing the boat's position.
        """

        self.get_logger().debug(f"received n {self.__wind_sensors_pub.topic}: {msg}")
        self.__start = False
        self.__boat_heading = msg.heading.heading
        self.__boat_speed = msg.speed.speed


# class MockWindSensor(Node):
#     """Publishes mock apparent wind speed and direction at a fixed rate.

#     See the module docstring for topics and parameters.
#     """

#     def __init__(self):
#         """Initialize timers, publishers, subscriptions, and declare parameters."""
#         super().__init__("mock_wind_sensor")

#         # Declare ROS parameters (qos depth and publish period)
#         self.declare_parameters(
#             namespace="",
#             parameters=[
#                 ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
#                 ("true_wind_speed_kmph", 10.0),
#                 ("true_wind_direction_deg", 30),  # from the bow to the stern of the boat
#                 ("sd_speed", 1.0),
#                 ("mode", "variable"),  # set constant for fixing the value to a constant
#                 ("direction_kappa", 50.0),  # concentration for von Mises (higher = tighter)
#             ],
#         )

#         self.pub_period_sec = (
#             self.get_parameter("pub_period_sec").get_parameter_value().double_value
#         )

#         # Mock wind sensor timer
#         self.__mock_wind_sensor_timer = self.create_timer(
#             timer_period_sec=self.pub_period_sec, callback=self.mock_wind_sensor_callback
#         )

#         # Mock wind sensor publisher initialization
#         self.__wind_sensors_pub = self.create_publisher(
#             msg_type=ci.WindSensor,
#             topic="filtered_wind_sensor",
#             qos_profile=10,
#         )

#         self.__gps_sub = self.create_subscription(
#             msg_type=ci.GPS, topic="gps", callback=self.gps_callback, qos_profile=10
#         )

#         self.__last_direction = int(
#             self.get_parameter("true_wind_direction_deg").get_parameter_value().double_value
#         )

#         self.__last_speed = ci.HelperSpeed(
#             speed=self.get_parameter("true_wind_speed_kmph").get_parameter_value().double_value
#         )

#         self.__ticks = random.randint(60, 120)
#         self.__ticks_so_far = 0
#         self.__start = True
#         self.__boat_heading = sc.START_HEADING
#         self.__boat_speed = sc.MEAN_SPEED

#     def mock_wind_sensor_callback(self) -> None:
#         """Timer callback to sample or hold wind values and publish a WindSensor message."""
#         self.get_latest_speed_and_direction_values()

#         wind_speed, direction = self.get_speed_and_direction()

#         msg = ci.WindSensor()
#         msg.speed, msg.direction = wind_speed, direction

#         self.get_logger().debug(f"Publishing to {self.__wind_sensors_pub.topic}: {msg}")
#         self.__wind_sensors_pub.publish(msg)

#     def get_speed_and_direction(self) -> tuple[ci.HelperSpeed, int]:
#         """Returns the latest speed and direction value based on the state of the node.
#         Returns:
#             tuple[ci.HelperSpeed, int]: Returns apparent wind in the format ready to be published.
#             The values returned are in standard wind units as mentioned in WindSensor.msg
#         """
#         aw_speed, aw_direction = wcs.get_apparent_wind(
#             self.__mean_direction_deg, self.__mean_wind_speed, self.__boat_heading, self.__boat_speed
#         )
#         direction_deg = np.degrees(aw_direction)
#         direction_value = int(
#             wcs.global_to_boat_coordinate(self.__boat_heading, cs.bound_to_180(direction_deg))
#         )
#         if self.__mode == "constant":
#             return ci.HelperSpeed(speed=aw_speed), direction_value

#         if self.__ticks_so_far < self.__ticks:
#             self.__ticks_so_far += 1
#             return self.__last_speed, self.__last_direction

#         k = (self.__mean_wind_speed / self.__sd_wind_speed) ** 2
#         theta = (self.__sd_wind_speed**2) / self.__mean_wind_speed

#         true_wind_speed = gamma.rvs(a=k, scale=theta)

#         # Convert true wind to apparent wind (global frame)
#         mean_aw_direction, mean_aw_speed = wcs.get_apparent_wind(
#             self.__mean_direction_deg, true_wind_speed,  self.__boat_heading, self.__boat_speed
#         )

#         mean_aw_dir_rad = np.radians(mean_aw_direction)
#         direction_rad = vonmises.rvs(kappa=self.__direction_kappa, loc=mean_aw_dir_rad, size=1)[0] # noqa
#         direction_deg = np.degrees(direction_rad)

#         speed_value = ci.HelperSpeed(speed=mean_aw_speed)
#         direction_value = int(
#             wcs.global_to_boat_coordinate(self.__boat_heading, cs.bound_to_180(direction_deg))
#         )

#         self._last_speed = speed_value
#         self.__last_direction = direction_value

#         return speed_value, direction_value

#     def get_latest_speed_and_direction_values(self) -> None:
#         """Updates mean wind speed and direction with the latest values from ROS parameters."""

#         self.__mean_wind_speed = (
#             self.get_parameter("true_wind_speed_kmph").get_parameter_value().double_value
#         )

#         self.__mean_direction_deg = (
#             self.get_parameter("true_wind_direction_deg").get_parameter_value().integer_value
#         )

#         self.__sd_wind_speed = self.get_parameter("sd_speed").get_parameter_value().double_value

#         self.__mode = self.get_parameter("mode").get_parameter_value().string_value
#         self.__direction_kappa = (
#             self.get_parameter("direction_kappa").get_parameter_value().double_value
#         )

#     def gps_callback(self, msg: ci.GPS) -> None:
#         """Callback function for the GPS subscription. Updates the boat's position.

#         Args:
#             msg (ci.GPS): The GPS message containing the boat's position.
#         """

#         self.get_logger().debug(f"received n {self.__wind_sensors_pub.topic}: {msg}")
#         self.__start = False
#         self.__boat_heading = msg.heading.heading
#         self.__boat_speed = msg.speed.speed


def main(args=None):
    rclpy.init(args=args)
    mock_wind_sensor = MockWindSensor()

    rclpy.spin(node=mock_wind_sensor)

    mock_wind_sensor.destroy_node()
    rclpy.shutdown()
