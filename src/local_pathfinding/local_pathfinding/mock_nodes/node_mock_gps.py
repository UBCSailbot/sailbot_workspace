"""
Mock class for the GPS. Publishes basic GPS data to the ROS network.
"""

import math

import custom_interfaces.msg as ci
import numpy as np
import rclpy
from geopy.distance import great_circle
from rclpy.node import Node

from local_pathfinding.wind_coord_systems import get_true_wind
import shared_constants as sc


BOATSPEEDS = np.array(
    [
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0.4, 1.1, 3.2, 3.7, 2.8],
        [0, 0.3, 1.9, 3.7, 9.3, 13.0, 9.2],
        [0, 0.9, 3.7, 7.4, 14.8, 18.5, 13.0],
        [0, 1.3, 5.6, 9.3, 18.5, 24.1, 18.5],
    ]
)

WINDSPEEDS = [0, 9.3, 18.5, 27.8, 37.0]  # The row labels
ANGLES = [0, 20, 30, 45, 90, 135, 180]  # The column labels


def get_sailbot_speed(
    heading: float, apparent_wind_direction: float, apparent_wind_speed: float, boat_speed: float
) -> float:
    """Get an estimated boat speed given the wind conditions and the speed of the boat from the
    previous iteration.

    Returns:
        float: the interpolated boat speed
    """

    true_wind_direction, true_wind_speed = get_true_wind(
        apparent_wind_direction,
        apparent_wind_speed,
        heading,
        boat_speed,
    )

    # Get the sailing angle: [0, 180]
    sailing_angle = abs(heading - true_wind_direction)
    sailing_angle = min(sailing_angle, 360 - sailing_angle)

    # Find the nearest windspeed values above and below the true windspeed
    lower_windspeed_index = max([i for i, ws in enumerate(WINDSPEEDS) if ws <= true_wind_speed])
    upper_windspeed_index = (
        lower_windspeed_index + 1
        if lower_windspeed_index < len(WINDSPEEDS) - 1
        else lower_windspeed_index
    )

    # Find the nearest angle values above and below the sailing angle
    lower_angle_index = max([i for i, ang in enumerate(ANGLES) if ang <= sailing_angle])
    upper_angle_index = (
        lower_angle_index + 1 if lower_angle_index < len(ANGLES) - 1 else lower_angle_index
    )

    # Find the maximum angle and maximum windspeed based on the actual data in the table
    max_angle = max(ANGLES)
    max_windspeed = max(WINDSPEEDS)

    # Handle the case of maximum angle (use the dynamic max_angle)
    if upper_angle_index == len(ANGLES) - 1:
        lower_angle_index = ANGLES.index(max_angle) - 1
        upper_angle_index = ANGLES.index(max_angle)

    # Handle the case of the maximum windspeed (use the dynamic max_windspeed)
    if upper_windspeed_index == len(WINDSPEEDS) - 1:
        lower_windspeed_index = WINDSPEEDS.index(max_windspeed) - 1
        upper_windspeed_index = WINDSPEEDS.index(max_windspeed)

    # Perform linear interpolation
    lower_windspeed = WINDSPEEDS[lower_windspeed_index]
    upper_windspeed = WINDSPEEDS[upper_windspeed_index]
    lower_angle = ANGLES[lower_angle_index]
    upper_angle = ANGLES[upper_angle_index]

    boat_speed_lower = BOATSPEEDS[lower_windspeed_index][lower_angle_index]
    boat_speed_upper = BOATSPEEDS[upper_windspeed_index][lower_angle_index]

    interpolated_1 = boat_speed_lower + (true_wind_speed - lower_windspeed) * (
        boat_speed_upper - boat_speed_lower
    ) / (upper_windspeed - lower_windspeed)

    boat_speed_lower = BOATSPEEDS[lower_windspeed_index][upper_angle_index]
    boat_speed_upper = BOATSPEEDS[upper_windspeed_index][upper_angle_index]

    interpolated_2 = boat_speed_lower + (true_wind_speed - lower_windspeed) * (
        boat_speed_upper - boat_speed_lower
    ) / (upper_windspeed - lower_windspeed)

    interpolated_value = interpolated_1 + (sailing_angle - lower_angle) * (
        interpolated_2 - interpolated_1
    ) / (upper_angle - lower_angle)

    return interpolated_value


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

        self.__mean_speed = sc.MEAN_SPEED
        self.__current_location = sc.START_POINT
        self.__heading = sc.START_HEADING

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
        apparent_wind_speed: float = msg.speed.speed
        apparent_wind_direction: float = msg.direction
        self.__mean_speed = ci.HelperSpeed(
            speed=get_sailbot_speed(
                heading=self.__heading.heading,
                apparent_wind_direction=apparent_wind_direction,
                apparent_wind_speed=apparent_wind_speed,
                boat_speed=self.__mean_speed.speed,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    mock_gps = MockGPS()

    rclpy.spin(node=mock_gps)

    mock_gps.destroy_node()
    rclpy.shutdown()
