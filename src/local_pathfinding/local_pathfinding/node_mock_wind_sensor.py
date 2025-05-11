"""
Mock class for the wind sensor. Publishes basic wind data to the ROS network.
"""

import custom_interfaces.msg as ci
import numpy as np
import rclpy
from local_pathfinding.coord_systems import bound_to_180
from rclpy.node import Node
from scipy.stats import vonmises, weibull_min


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
            __mean_direction (int): Mean direction of the wind during July.
            This parameter can be set during runtime using
            'ros2 param set /mock_wind_sensor mean_direction {value}'
        """
        super().__init__("mock_wind_sensor")

        # Declare ROS parameters (qos depth and publish period)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("mean_wind_speed", 7.0),
                ("mean_direction", 0),
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

        # Shape=2 (typical for wind under normal calm conditions), Scale=7 knots (mean)
        scale = self.__mean_wind_speed
        wind_speed_knots = weibull_min.rvs(c=10, scale=scale, size=1)
        return ci.HelperSpeed(speed=wind_speed_knots[0])

    def get_direction_value(self) -> int:
        """Generates a random wind direction based on a von Mises distribution centered around the
        mean. This distribution is a circular distribution, which is perfect for wind direction.
        Returns:
            int: The wind direction in degrees.
        """

        direction = int(np.degrees(vonmises.rvs(kappa=55, loc=self.__mean_direction, size=1)))
        return int(bound_to_180(direction))

    def get_latest_speed_and_direction_values(self) -> None:
        """Updates mean wind speed and direction with the latest values from ROS parameters."""

        self.__mean_wind_speed = (
            self.get_parameter("mean_wind_speed").get_parameter_value().double_value
        )

        self.__mean_direction = (
            self.get_parameter("mean_direction").get_parameter_value().integer_value
        )


def main(args=None):
    rclpy.init(args=args)
    mock_wind_sensor = MockWindSensor()

    rclpy.spin(node=mock_wind_sensor)

    mock_wind_sensor.destroy_node()
    rclpy.shutdown()
