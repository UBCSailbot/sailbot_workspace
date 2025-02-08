#!/usr/bin/env python3

"""The ROS node for the wingsail controller."""

import math

import rclpy
import rclpy.utilities
from custom_interfaces.msg import GPS, SailCmd, WindSensor
from rclpy.node import Node

from controller.common.constants import (
    CHORD_WIDTH_MAIN_SAIL,
    KINEMATIC_VISCOSITY,
    REYNOLDS_NUMBER_ALPHA_TABLE,
)
from controller.common.lut import LUT
from controller.wingsail.controllers import WingsailController


def main(args=None):
    rclpy.init(args=args)
    node = WingsailControllerNode()
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()


class WingsailControllerNode(Node):
    """
    A ROS node that controls the trim tab angle wingsail of the boat. The objective
    of the wingsail controller is to maintain the wingsail at a desired angle of attack
    while optimizing for speed by maximizing the lift-to-drag ratio of the wingsail.

    Subscriptions:
        __filtered_wind_sensors_sub (Subscription): Subscribes to the filtered_wind_sensor topic
        __gps_sub (Subscription): Subscribes to the gps topic

    Publishers:
        TO BE ADDED
    """

    def __init__(self):
        """Initializes an instance of this class."""
        super().__init__("wingsail_ctrl_node")

        self.get_logger().debug("Initializing node...")
        self.__init_private_attributes()
        self.__declare_ros_parameters()
        self.__init_subscriptions()
        self.__init_publishers()
        self.__init_timer_callbacks()
        self.get_logger().debug("Node initialization complete. Starting execution...")

    def __init_private_attributes(self):
        """Initializes private attributes of this class that are not initialized anywhere else
        during the initialization process.
        """
        self.__trim_tab_angle = 0.0
        self.__filtered_wind_sensor = WindSensor()
        self.__gps = GPS()
        # pull hardcoded table from the right place later...
        # right location should be config.py
        lut = LUT(REYNOLDS_NUMBER_ALPHA_TABLE)
        self.__wingsailController = WingsailController(
            CHORD_WIDTH_MAIN_SAIL, KINEMATIC_VISCOSITY, lut
        )

    def __declare_ros_parameters(self):
        """Declares ROS parameters from the global configuration file that will be used in this
        node. This node will monitor for any changes to these parameters during execution and will
        update itself accordingly.
        """
        self.get_logger().debug("Declaring ROS parameters...")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("reynolds_number", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("angle_of_attack", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("apparent_wind_threshold", rclpy.Parameter.Type.DOUBLE),
                ("scaling_coef", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        # TODO Revisit this debug statement. It might get ugly for args with complicated structures
        all_parameters = self._parameters
        for name, parameter in all_parameters.items():
            value_str = str(parameter.value)
            self.get_logger().debug(f"Got parameter {name} with value {value_str}")

    def __init_subscriptions(self):
        """Initializes the subscriptions of this node. Subscriptions pull data from other ROS
        topics for further usage in this node. Data is pulled from subscriptions periodically via
        callbacks, which are registered upon subscription initialization.
        """
        # TODO Implement this function by subscribing to topics that give the desired input data
        # Callbacks for each subscriptions should be defined as private methods of this class
        self.get_logger().debug("Initializing subscriptions...")

        self.__filtered_wind_sensor_sub = self.create_subscription(
            msg_type=WindSensor,
            topic="filtered_wind_sensor",
            callback=self.__filtered_wind_sensor_sub_callback,
            qos_profile=1,
        )

        self.__gps_sub = self.create_subscription(
            msg_type=GPS,
            topic="gps",
            callback=self.__gps_sub_callback,
            qos_profile=1,
        )

    def __init_publishers(self):
        """Initializes the publishers of this node. Publishers update ROS topics so that other ROS
        nodes in the system can utilize the data produced by this node.
        """

        self.get_logger().debug("Initializing publishers...")
        self.__trim_tab_angle_pub = self.create_publisher(
            msg_type=SailCmd,
            # TODO change "topic" from a magic string to a constant similar to how its done in the
            # boat simulator
            topic="sail_cmd",
            qos_profile=1,
        )

    def __init_timer_callbacks(self):
        """Initializes the timer callbacks of this node. Timer callbacks are registered to be
        called at the specified frequency."""

        self.get_logger().debug("Initializing timer callbacks...")

        # Publishing data to ROS topics
        self.create_timer(
            timer_period_sec=self.pub_period,
            callback=self.__publish,
        )

    # PUBLISHER CALLBACKS
    def __publish(self):
        """Publishes a SailCmd message with the trim tab angle using the designated publisher.
        It also logs information about the publication to the logger."""
        msg = SailCmd()

        apparent_speed = self.__filtered_wind_sensor.speed.speed
        apparent_direction = self.__filtered_wind_sensor.direction
        apparent_threshold = (
            self.get_parameter("apparent_wind_threshold").get_parameter_value().double_value
        )

        # Sets trim tab angle, scales if apparent wind speed is above threshold
        self.__trim_tab_angle = self.__wingsailController.get_trim_tab_angle(
            apparent_speed, apparent_direction
        )
        if apparent_speed > apparent_threshold:
            coef = self.get_parameter("scaling_coef").get_parameter_value().double_value
            speed_difference = apparent_threshold - apparent_speed
            self.__trim_tab_angle = self.__trim_tab_angle * math.exp(
                -1 * coef * abs(speed_difference)
            )

        msg.trim_tab_angle_degrees = self.__trim_tab_angle

        self.__trim_tab_angle_pub.publish(msg)

        self.get_logger().info(
            f"Published to {self.__trim_tab_angle_pub.topic} \
                the following angle: {msg.trim_tab_angle_degrees}"
        )

    @property
    def pub_period(self) -> float:
        return self.get_parameter("pub_period_sec").get_parameter_value().double_value

    @property
    def trim_tab_angle(self) -> float:
        return self.__trim_tab_angle

    def __filtered_wind_sensor_sub_callback(self, msg: WindSensor) -> None:
        """Stores the latest filtered wind sensor data

        Args:
            msg (WindSensor): Filtered wind sensor data from CanTrxRosIntf.
        """
        self.__filtered_wind_sensor = msg
        self.get_logger().info(f"Received data from {self.__filtered_wind_sensor_sub.topic}")

    def __gps_sub_callback(self, msg: GPS) -> None:
        """Stores the latest gps data

        Args:
            msg (GPS): gps data from CanTrxRosIntf.
        """
        self.__gps = msg
        self.get_logger().info(f"Received data from {self.__gps_sub.topic}")


if __name__ == "__main__":
    main()
