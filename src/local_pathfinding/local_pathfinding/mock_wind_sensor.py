"""
Mock class for the wind sensor. Publishes basic wind data to the ROS network.
"""

import rclpy
import numpy as np
import custom_interfaces.msg as ci
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
            __mean_direction (int): Mean direction of the wind during July.
        """
        super().__init__("mock_wind_sensor")

        # Declare ROS parameters (qos depth and publish period)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
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

        self.__mean_wind_speed = 7.0  # mean wind speed in pacific ocean near Vancouver in July
        self.__mean_direction = 0  # in degrees, mean direction of wind during July

    def mock_wind_sensor_callback(self) -> None:
        """Callback function for the mock wind sensor timer. Publishes mock wind data to the ROS
        network.
        """
        wind_speed_knots = self.get_mock_wind_speed()
        direction = self.get_direction_value()

        msg = ci.WindSensor()
        msg.speed, msg.direction = wind_speed_knots, direction

        self.get_logger().info(f"Publishing to {self.__wind_sensors_pub.topic}: {msg}")
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
<<<<<<< HEAD
        wind_speed_knots = weibull_min.rvs(c=10, scale=scale, size=1)
=======
        wind_speed_knots = weibull_min.rvs(c=2, scale=scale, size=1)
>>>>>>> main
        return ci.HelperSpeed(speed=wind_speed_knots[0])

    def get_direction_value(self) -> int:
        """Generates a random wind direction based on a von Mises distribution centered around the
        mean. This distribution is a circular distribution, which is perfect for wind direction.
        Returns:
            int: The wind direction in degrees.
        """

<<<<<<< HEAD
        direction = int(np.degrees(vonmises.rvs(kappa=55, loc=self.__mean_direction, size=1)))
=======
        direction = int(np.degrees(vonmises.rvs(kappa=10, loc=self.__mean_direction, size=1)))
>>>>>>> main
        return self.ensure_correct_range(direction)

    def ensure_correct_range(self, direction_value: int) -> int:
        """Helper function to ensure the wind direction is in the range (-180, 180].
        Args:
            direction_value (int): The wind direction in degrees which may not be in the correct
            range.
        Returns:
            int: The corrected wind direction in degrees.
        """
        if direction_value > 180:
            return (360 - (direction_value % 360)) * -1
        return direction_value


def main(args=None):
    rclpy.init(args=args)
    mock_wind_sensor = MockWindSensor()

    rclpy.spin(node=mock_wind_sensor)

    mock_wind_sensor.destroy_node()
    rclpy.shutdown()
