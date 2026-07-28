#!/usr/bin/env python3

"""The ROS node for the wingsail controller."""


import rclpy
import rclpy.utilities
from rclpy.node import Node

from controller.common.constants import (
    CHORD_WIDTH_MAIN_SAIL,
    KINEMATIC_VISCOSITY,
    REYNOLDS_NUMBER_ALPHA_TABLE,
)
from controller.common.lut import LUT
from controller.wingsail.controllers import WingsailController
from custom_interfaces.msg import DesiredHeading, SailCmd, WindSensor


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
        __sail_sub (Subscription) : Subscribes to the desired heading

    Publishers:
        __trim_tab_angle_pub (Publisher): Publishes a SailCmd message with the trim tab angle from
        the __publish callback
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
        self.__sail = True
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
                ("apparent_wind_lower_threshold_kmph", rclpy.Parameter.Type.DOUBLE),
                ("apparent_wind_upper_threshold_kmph", rclpy.Parameter.Type.DOUBLE),
                ("apparent_wind_zero_threshold_kmph", rclpy.Parameter.Type.DOUBLE),
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

        self.__sail_sub = self.create_subscription(
            msg_type=DesiredHeading,
            topic="desired_heading",
            callback=self.__sail_sub_callback,
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

        apparent_wind_speed_kmph = self.__filtered_wind_sensor.speed.speed
        apparent_wind_direction_deg = self.__filtered_wind_sensor.direction
        apparent_wind_lower_threshold_kmph = (
            self.get_parameter("apparent_wind_lower_threshold_kmph")
            .get_parameter_value()
            .double_value
        )
        apparent_wind_upper_threshold_kmph = (
            self.get_parameter("apparent_wind_upper_threshold_kmph")
            .get_parameter_value()
            .double_value
        )
        apparent_wind_zero_threshold_kmph = (
            self.get_parameter("apparent_wind_zero_threshold_kmph")
            .get_parameter_value()
            .double_value
        )

        self.__trim_tab_angle = self.__wingsailController.get_trim_tab_angle(
            apparent_wind_speed_kmph, apparent_wind_direction_deg
        )

        # Gets scaling factor based on wind speed thresholds
        scaling_coef = 1
        if (
            apparent_wind_speed_kmph > apparent_wind_lower_threshold_kmph
            and apparent_wind_speed_kmph < apparent_wind_upper_threshold_kmph
        ):
            threshold_difference_kmph = (
                apparent_wind_upper_threshold_kmph - apparent_wind_lower_threshold_kmph
            )
            scaling_coef = (
                -1
                * (apparent_wind_speed_kmph - apparent_wind_lower_threshold_kmph)
                / threshold_difference_kmph
                + 1
            )
        elif apparent_wind_speed_kmph < apparent_wind_zero_threshold_kmph:
            scaling_coef = 0
        elif apparent_wind_speed_kmph >= apparent_wind_upper_threshold_kmph:
            scaling_coef = 0

        if self.__sail:
            self.__trim_tab_angle = scaling_coef * self.__trim_tab_angle
        else:
            self.__trim_tab_angle = 0.0

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
    def mock_desired_heading(self):
        return self.get_parameter("mock_desired_heading").get_parameter_value().bool_value

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

    def __sail_sub_callback(self, msg: DesiredHeading) -> None:
        """Stores the latest desired heading data. We use the sail message to decide when the boat
        should keep moving or needs to enter idle mode.
        Args:
            msg (DesiredHeading): desired heading data from CanTrxRosIntf.
        """
        self.__sail = msg.sail
        self.get_logger().info(f"Received data from {self.__sail_sub.topic}")


if __name__ == "__main__":
    main()
