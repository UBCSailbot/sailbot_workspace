"""
Mock class for the wind sensor. Publishes basic wind data to the ROS network.
"""

import custom_interfaces.msg as ci
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.stats import vonmises, gamma
import random
import math

import local_pathfinding.coord_systems as cs


class MockWindSensor(Node):

    def __init__(self):
        """Initialize the MockWindSensor class. The class is used to publish mock filtered wind
        data to the ROS network.

        Args:
            Node (Node): The ROS node that the class will run on. Superclass of MockWindSensor.

        Attributes:
            __mock_wind_sensor_timer (Timer): Timer to call the mock wind sensor callback function.
            __wind_sensors_pub (Publisher): Publisher for the filtered wind sensor data.
            __mean_wind_speed (float): Mean wind speed in the Pacific Ocean near Vancouver in July.
            This parameter can be set during runtime using
            'ros2 param set /mock_wind_sensor mean_wind_speed {value}'
            __mean_direction (int): Mean direction of the wind during July. This is true wind.
            We publish apparent wind.
            This parameter can be set during runtime using.
            'ros2 param set /mock_wind_sensor mean_direction {value}'
            __last_direction: last wind direction that was set
        """
        super().__init__("mock_wind_sensor")

        # Declare ROS parameters (qos depth and publish period)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("mean_wind_speed", 16.0),
                ("mean_direction", 30),  # from the bow to the stern of the boat
                ("sd_speed", 1.0),
                ("mode", "variable")  # set constant for fixing the value to a constant
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

        self.__last_direction = int(
            self.get_parameter("mean_direction").get_parameter_value().double_value
        )

        self.__ticks = random.randint(60, 120)
        self.__ticks_so_far = 0

    def mock_wind_sensor_callback(self) -> None:
        """Callback function for the mock wind sensor timer. Publishes mock wind data to the ROS
        network.
        """

        self.get_latest_speed_and_direction_values()
        wind_speed_knots = self.get_mock_wind_speed()
        direction = self.get_direction_value()

        msg = ci.WindSensor()
        msg.speed, msg.direction = wind_speed_knots, direction

        self.get_logger().debug(f"Publishing to {self.__wind_sensors_pub.topic}: {msg}")
        self.__wind_sensors_pub.publish(msg)

    def get_mock_wind_speed(self) -> ci.HelperSpeed:
        """Generates a random wind speed based on a Weibull distribution centered around the mean.
        This distribution is a good fit for wind speed because is naturally positively skewed
        and has a long tail.
        Returns:
            HelperSpeed: The wind speed in knots.
        """

        if self.__mode == "constant":
            return ci.HelperSpeed(speed=self.__mean_wind_speed)

        # Shape=2 (typical for wind under normal calm conditions), Scale=7 knots (mean)
        mean = self.__mean_wind_speed
        sd = self.__sd_wind_speed

        k = (mean / sd) ** 2
        theta = (sd ** 2) / mean

        # Generate a random wind speed
        wind_speed_knots = gamma.rvs(a=k, scale=theta)
        return ci.HelperSpeed(speed=abs(wind_speed_knots))

    def get_direction_value(self) -> int:
        """Generates a random wind direction based on a von Mises distribution centered around the
        mean. This distribution is a circular distribution, which is perfect for wind direction.
        Returns:
            int: The wind direction in degrees.
        """
        if self.__mode == "constant":
            return self.__mean_direction

        if self.__ticks_so_far < self.__ticks:
            self.__ticks_so_far += 1
            return self.__last_direction

        self.__ticks = random.randint(60, 120)
        self.__ticks_so_far = 0

        # Convert mean direction from degrees to radians for vonmises
        mean_direction_rad = np.radians(self.__mean_direction)
        direction_rad = vonmises.rvs(kappa=15, loc=mean_direction_rad, size=1)[0]
        direction_deg = np.degrees(direction_rad)
        result = int(cs.bound_to_180(direction_deg))
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

        self.__sd_wind_speed = (
            self.get_parameter("sd_speed").get_parameter_value().double_value
        )

        self.__mode = (
            self.get_parameter("mode").get_parameter_value().string_value
        )

    def get_apparent_wind(true_wind, boat_wind):
        tw_radians = math.radians(true_wind)
        bw_radians = math.radians(boat_wind)
        
        tw_x = math.cos(tw_radians)
        tw_y = 

def main(args=None):
    rclpy.init(args=args)
    mock_wind_sensor = MockWindSensor()

    rclpy.spin(node=mock_wind_sensor)

    mock_wind_sensor.destroy_node()
    rclpy.shutdown()
