"""
Mock class for the GPS. Publishes basic GPS data to the ROS network.
"""

import math

import custom_interfaces.msg as ci
import rclpy
from geopy.distance import great_circle
from rclpy.node import Node
from local_pathfinding.objectives import SpeedObjective


MEAN_SPEED = ci.HelperSpeed(speed=15.0)  # mean boat speed in kmph

START_POINT = ci.HelperLatLon(latitude=48.46, longitude=-125.1)  # Starting location of the mock
START_HEADING = ci.HelperHeading(heading=180.0)  # in degrees, heading of the boat


class MockGPS(Node):

    def __init__(self):
        """Initialize the MockGPS class. The class is used to publish mock mock gps
        data to the ROS network.

        Args:
            Node (Node): The ROS node that the class will run on.

        Attributes:
            __mock_gps_timer (Timer): Timer to call the mock gps callback function.
            __mock_gps_publisher (Publisher): Publisher for the gps data.
            __mean_speed (HelperSpeed): Constant speed of the boat.
            __current_location (HelperLatLon): Current location of the boat.
            __mean_heading (HelperHeading): Constant heading of the boat.
        """
        super().__init__("mock_gps")

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

        # callback for wind sensor sub to update the boat speed
        self.__mock_wind_sensor_sub = self.create_subscription(
            msg_type=ci.WindSensor,
            topic="filtered_wind_sensor",
            callback=self.wind_sensor_callback,
            qos_profile=10,
        )

        self.__mean_speed = MEAN_SPEED
        self.__current_location = START_POINT
        self.__heading = START_HEADING

    def mock_gps_callback(self) -> None:
        """Callback function for the mock GPS timer. Publishes mock gps data to the ROS
        network.
        """
        self.get_next_location()
        msg: ci.GPS = ci.GPS(
            lat_lon=self.__current_location, speed=self.__mean_speed, heading=self.__heading
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

    def get_next_location(self) -> None:
        """Get the next location by following the great circle. Assumes constant speed and heading"""  # noqa
        # distance travelled = speed * calback time (s)
        distance_km: float = self.__mean_speed.speed * (self.pub_period_sec / 3600)
        start: tuple[float, float] = (
            self.__current_location.latitude,
            self.__current_location.longitude,
        )
        radian_angle = math.radians(self.__heading.heading)
        destination = great_circle(kilometers=distance_km).destination(start, radian_angle)
        self.__current_location = ci.HelperLatLon(
            latitude=destination.latitude, longitude=destination.longitude
        )

    def desired_heading_callback(self, msg: ci.DesiredHeading):
        """Callback for topic desired heading

        Args:
            msg (ci.DesiredHeading): The desired heading for the boat.
        """
        self._logger.debug(f"Received data from {self.__desired_heading_sub.topic}: {msg}")
        self.__heading = msg.heading

    def wind_sensor_callback(self, msg: ci.WindSensor):
        """Callback for the data published by the wind sensor node (mock)
        Calls SpeedObjective.get_sailbot_speed to get the updated mean speed

        Args:
            msg (ci.WindSensor): the wind sensor speed
        """

        self._logger.debug(f"Received data from {self.__mock_wind_sensor_sub.topic}: {msg}")
        wind_speed: float = msg.speed.speed
        wind_direction: float = msg.direction
        self.__mean_speed = ci.HelperSpeed(
            speed=SpeedObjective.get_sailbot_speed(
                    heading=self.__heading.heading,
                    wind_direction=wind_direction,
                    wind_speed=wind_speed,
                )
        )


def main(args=None):
    rclpy.init(args=args)
    mock_gps = MockGPS()

    rclpy.spin(node=mock_gps)

    mock_gps.destroy_node()
    rclpy.shutdown()
