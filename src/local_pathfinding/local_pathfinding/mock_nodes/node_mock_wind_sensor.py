"""Mock wind sensor node for ROS 2.

This node publishes mock data that mimics the boat's wind sensors.

It takes **true wind** inputs in the **global frame** via ROS parameters and publishes
**apparent wind** outputs in the **boat frame** on ``filtered_wind_sensor``.

Important design notes:

Node name:
* ``mock_wind_sensor``

Publishes:
* topic: ``filtered_wind_sensor`` (custom_interfaces/WindSensor)

Subscribes:
* topic: ``gps`` (custom_interfaces/GPS) for boat heading and speed

Parameters:
* ``pub_period_sec`` (double, required): publish period (seconds)
* ``true_wind_speed_kmph`` (double, default: 10.0): true wind speed (kmph)
* ``true_wind_direction_deg`` (int, default: 90): true wind direction in global frame (deg)
    * Valid range: (-180, 180]
"""

import custom_interfaces.msg as ci
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from typing import List
import local_pathfinding.mock_nodes.shared_constants as sc
import local_pathfinding.wind_coord_systems as wcs


def _validate_true_wind_direction_deg(value: int) -> None:
    """Validate direction is in (-180, 180]."""
    if not (-180 < value <= 180):
        raise ValueError(
            f"true_wind_direction_deg must be in (-180, 180]; got {value}"
        )


class MockWindSensor(Node):
    def __init__(self):
        super().__init__("mock_wind_sensor")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("true_wind_speed_kmph", 10.0),
                ("true_wind_direction_deg", 90),  # from the bow to the stern of the boat
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

        # Cached parameter-backed values (updated through on-set-parameters callback).
        self.__true_wind_speed_kmph = float(
            self.get_parameter("true_wind_speed_kmph").value
        )
        self.__true_wind_direction_deg = int(
            self.get_parameter("true_wind_direction_deg").value
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

    def mock_wind_sensor_callback(self):
        aw_speed_kmph, aw_direction_rad = wcs.get_apparent_wind(
            self.__true_wind_direction_deg,
            self.__true_wind_speed_kmph,
            self.__boat_heading,
            self.__boat_speed,
        )
        aw_direction_boat_coord_deg = wcs.global_to_boat_coordinate(
            self.__boat_heading,
            np.degrees(aw_direction_rad),
        )
        msg = ci.WindSensor(
            speed=ci.HelperSpeed(speed=aw_speed_kmph),
            direction=int(aw_direction_boat_coord_deg),
        )
        self.get_logger().debug(f"Publishing to {self.__wind_sensors_pub.topic}: {msg}")
        self.__wind_sensors_pub.publish(msg)

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        """ROS2 parameter update callback.

        Applies updates to true wind speed/direction. Values take effect on the next publish tick.
        """
        try:
            for p in params:
                if p.name == "true_wind_direction_deg":
                    new_direction_deg = int(p.value)
                    _validate_true_wind_direction_deg(new_direction_deg)
                    self.__true_wind_direction_deg = new_direction_deg
                else:
                    self.__true_wind_speed_kmph = p.value
            return SetParametersResult(successful=True)
        except Exception:
            reason = f"Please enter the direction in (-180, 180]. Got {p.value}."
            return SetParametersResult(successful=False, reason=reason)

    def gps_callback(self, msg: ci.GPS) -> None:
        """Callback function for the GPS subscription. Updates the boat's position.

        Args:
            msg (ci.GPS): The GPS message containing the boat's position.
        """

        self.get_logger().debug(f"received n {self.__wind_sensors_pub.topic}: {msg}")
        self.__boat_heading = msg.heading.heading
        self.__boat_speed = msg.speed.speed


def main(args=None):
    rclpy.init(args=args)
    mock_wind_sensor = MockWindSensor()

    rclpy.spin(node=mock_wind_sensor)

    mock_wind_sensor.destroy_node()
    rclpy.shutdown()
