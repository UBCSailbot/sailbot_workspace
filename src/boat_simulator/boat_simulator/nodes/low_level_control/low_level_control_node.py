#!/usr/bin/env python3

"""The ROS node for the low level controller emulation."""

from typing import Optional

import rclpy
import rclpy.utilities
from custom_interfaces.action import SimRudderActuation, SimSailTrimTabActuation
from custom_interfaces.action._sim_rudder_actuation import SimRudderActuation_Result
from custom_interfaces.action._sim_sail_trim_tab_actuation import (
    SimSailTrimTabActuation_Result,
)
from custom_interfaces.msg import GPS
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import CallbackGroup, MutuallyExclusiveCallbackGroup, Node
from rclpy.subscription import Subscription
from rclpy.timer import Rate

import boat_simulator.common.constants as Constants
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.low_level_control.controller import (
    RudderController,
    SailController,
)
from boat_simulator.nodes.low_level_control.decorators import (
    MutuallyExclusiveActionRoutine,
)


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


class LowLevelControlNode(Node):
    """Performs the low-level contoller emulation for the rudder and sail actuation mechanisms.

    This class uses a multithreaded executor. Refrain from using any synchronous calls within
    callbacks, or a deadlock is likely to occur.

    Subscriptions:
        gps_sub (Subscription): Subscribes to a `GPS` message.

    Action Servers:
        rudder_actuation_action_server (ActionServer): Performs the rudder actuation routine.
        sail_actuation_action_server (ActionServer): Performs the sail trim tab actuation routine.
    """

    def __init__(self):
        """Initializes an instance of this class."""
        super().__init__("low_level_control_node")

        self.get_logger().debug("Initializing node...")
        self.__init_private_attributes()
        self.__declare_ros_parameters()
        self.__init_callback_groups()
        self.__init_feedback_execution_rates()
        self.__init_subscriptions()
        self.__init_action_servers()
        self.get_logger().debug("Node initialization complete. Starting execution...")

    def __init_private_attributes(self):
        """Initializes private attributes of this class that are not initialized anywhere else
        during the initialization process.
        """
        self.__rudder_angle = 0
        self.__sail_trim_tab_angle = 0
        self._is_rudder_action_active = False
        self._is_sail_action_active = False
        self.__gps = None

    def __declare_ros_parameters(self):
        """Declares ROS parameters from the global configuration file that will be used in this
        node. This node will monitor for any changes to these parameters during execution and will
        update itself accordingly.
        """
        # TODO Update global YAML file with more configuration parameters and declare them here
        self.get_logger().debug("Declaring ROS parameters...")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("logging_throttle_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("info_log_throttle_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("qos_depth", rclpy.Parameter.Type.INTEGER),
                ("rudder.disable_actuation", rclpy.Parameter.Type.BOOL),
                ("rudder.fixed_angle_deg", rclpy.Parameter.Type.DOUBLE),
                ("rudder.actuation_execution_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("rudder.pid.kp", rclpy.Parameter.Type.DOUBLE),
                ("rudder.pid.ki", rclpy.Parameter.Type.DOUBLE),
                ("rudder.pid.kd", rclpy.Parameter.Type.DOUBLE),
                ("rudder.pid.buffer_size", rclpy.Parameter.Type.INTEGER),
                ("wingsail.disable_actuation", rclpy.Parameter.Type.BOOL),
                ("wingsail.fixed_angle_deg", rclpy.Parameter.Type.DOUBLE),
                ("wingsail.actuation_execution_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("wingsail.actuation_speed_deg_per_sec", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        # TODO Revisit this debug statement. It might get ugly for args with complicated structures
        all_parameters = self._parameters
        for name, parameter in all_parameters.items():
            value_str = str(parameter.value)
            self.get_logger().debug(f"Got parameter {name} with value {value_str}")

    def __init_callback_groups(self):
        """Initializes the callback groups. This node uses a multithreaded executor, so multiple
        callback groups are used to parallelize multiple action servers in the same node.

        Callbacks belonging to different groups may execute in parallel to each other. Learn more
        about executors and callback groups here:
        https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html#executors
        """
        self.get_logger().debug("Initializing callback groups...")
        self.__pub_sub_callback_group = MutuallyExclusiveCallbackGroup()
        self.__rudder_action_callback_group = MutuallyExclusiveCallbackGroup()
        self.__sail_action_callback_group = MutuallyExclusiveCallbackGroup()

    def __init_feedback_execution_rates(self):
        """Initializes rate objects used in this node to control how often a loop is executed
        within a callback.

        WARNING: Care should be taken when using rate objects to sleep within a callback. If using
        a single threaded executor, sleeping in a callback may block execution forever, causing a
        deadlock. Only sleep if the node executor is multithreaded.
        """
        self.get_logger().debug("Initializing rate objects...")
        self.__rudder_action_feedback_rate = self.create_rate(
            frequency=self.get_parameter("rudder.actuation_execution_period_sec")
            .get_parameter_value()
            .double_value,
            clock=self.get_clock(),
        )
        self.__sail_action_feedback_rate = self.create_rate(
            frequency=self.get_parameter("wingsail.actuation_execution_period_sec")
            .get_parameter_value()
            .double_value,
            clock=self.get_clock(),
        )

    def __init_subscriptions(self):
        """Initializes the subscriptions of this node. Subscriptions pull data from other ROS
        topics for further usage in this node. Data is pulled from subscriptions periodically via
        callbacks, which are registered upon subscription initialization.
        """
        self.get_logger().debug("Initializing subscriptions...")
        self.__gps_sub = self.create_subscription(
            msg_type=GPS,
            topic=Constants.LOW_LEVEL_CTRL_SUBSCRIPTIONS.GPS,
            callback=self.__gps_sub_callback,
            qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
            callback_group=self.pub_sub_callback_group,
        )

    def __init_action_servers(self):
        """Initializes the action servers of this node. Action servers perform a specified routine
        when a goal request comes from an action client.
        """
        self.get_logger().debug("Initializing action servers...")
        self.__rudder_actuation_action_server = ActionServer(
            node=self,
            action_type=SimRudderActuation,
            action_name=Constants.ACTION_NAMES.RUDDER_ACTUATION,
            execute_callback=self.__rudder_actuation_routine,
            callback_group=self.rudder_action_callback_group,
        )
        self.__sail_actuation_action_server = ActionServer(
            node=self,
            action_type=SimSailTrimTabActuation,
            action_name=Constants.ACTION_NAMES.SAIL_ACTUATION,
            execute_callback=self.__sail_actuation_routine,
            callback_group=self.sail_action_callback_group,
        )

    def __gps_sub_callback(self, msg: GPS):
        """Stores the latest GPS data.

        Args:
            msg (GPS): The GPS data from the physics engine.
        """
        self.get_logger().info(
            f"Received data from {self.gps_sub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )
        self.__gps = msg

    @MutuallyExclusiveActionRoutine(action_type=SimRudderActuation)
    def __rudder_actuation_routine(
        self, goal_handle: ServerGoalHandle
    ) -> Optional[SimRudderActuation_Result]:
        """The rudder actuation action server routine. Given a desired heading as the goal, the
        rudder is actuated until the desired heading is reached or the action times out.

        Args:
            goal_handle (ServerGoalHandle): The server goal specified by the client.

        Returns:
            Optional[SimRudderActuation_Result]: The result message if successful.
        """
        self.get_logger().debug("Beginning rudder actuation...")
        all_parameters = self._parameters

        current_heading = self.__gps.heading
        desired_heading = goal_handle.request.desired_heading
        current_control_ang = self.rudder_angle
        time_step = all_parameters["rudder.actuation_execution_period_sec"].value
        kp = all_parameters["rudder.pid.kp"].value
        cp = all_parameters["rudder.pid.kp"].value  # not sure if this is the right value to use
        control_speed = 10  # not sure if this is the right value to use

        rudder_controller = RudderController(
            current_heading, desired_heading, current_control_ang, time_step, kp, cp, control_speed
        )

        # TODO Placeholder loop. Replace with PID ctrl once implemented.
        feedback_msg = SimRudderActuation.Feedback()
        while self.is_rudder_action_active and not rudder_controller.is_target_reached:
            rudder_controller.update_state()
            feedback_msg.rudder_angle = float(rudder_controller.current_control_ang)
            self.get_logger().debug(
                f"Rudder Action Server feedback: {rudder_controller.current_control_ang}"
            )
            self.get_logger().debug(f"Is Rudder Action Active? {self.is_rudder_action_active}")
            goal_handle.publish_feedback(feedback=feedback_msg)
            self.rudder_action_feedback_rate.sleep()

        goal_handle.succeed()

        result = SimRudderActuation.Result()
        result.remaining_angular_distance = 0.0
        return result

    @MutuallyExclusiveActionRoutine(action_type=SimSailTrimTabActuation)
    def __sail_actuation_routine(
        self, goal_handle: ServerGoalHandle
    ) -> Optional[SimSailTrimTabActuation_Result]:
        """The sail actuation action server routine. Given a desired angular position as the goal,
        the trim tab is actuated until the desired position is reached or the action times out.

        Args:
            goal_handle (ServerGoalHandle): The server goal specified by the client.

        Returns:
            Optional[SimSailTrimTabActuation_Result]: The result message if successful.
        """

        self.get_logger().debug("Beginning sail actuation...")
        all_parameters = self._parameters

        target_angle = goal_handle.request.desired_angular_position
        current_angle = self.sail_trim_tab_angle
        time_step = all_parameters["wingsail.actuation_execution_period_sec"].value
        control_speed = all_parameters["wingsail.actuation_speed_deg_per_sec"].value
        sail_controller = SailController(target_angle, current_angle, time_step, control_speed)

        # TODO Placeholder loop. Replace with sail ctrl once implemented.
        feedback_msg = SimSailTrimTabActuation.Feedback()
        while self.is_sail_action_active and not sail_controller.is_target_reached:
            sail_controller.update_state()
            feedback_msg.current_angular_position = sail_controller.current_control_ang
            self.get_logger().debug(
                f"Sail Action Server feedback: {sail_controller.current_control_ang}"
            )
            self.get_logger().debug(f"Is Sail Action Active? {self.is_sail_action_active}")
            goal_handle.publish_feedback(feedback=feedback_msg)
            self.sail_action_feedback_rate.sleep()

        goal_handle.succeed()

        result = SimSailTrimTabActuation.Result()
        result.remaining_angular_distance = 0.0
        return result

    @property
    def is_multithreading_enabled(self) -> bool:
        return self.__is_multithreading_enabled

    @property
    def pub_sub_callback_group(self) -> CallbackGroup:
        return self.__pub_sub_callback_group

    @property
    def rudder_action_callback_group(self) -> CallbackGroup:
        return self.__rudder_action_callback_group

    @property
    def sail_action_callback_group(self) -> CallbackGroup:
        return self.__sail_action_callback_group

    @property
    def rudder_actuation_action_server(self) -> ActionServer:
        return self.__rudder_actuation_action_server

    @property
    def sail_actuation_action_server(self) -> ActionServer:
        return self.__sail_actuation_action_server

    @property
    def rudder_action_feedback_rate(self) -> Rate:
        return self.__rudder_action_feedback_rate

    @property
    def sail_action_feedback_rate(self) -> Rate:
        return self.__sail_action_feedback_rate

    @property
    def is_rudder_action_active(self) -> bool:
        return self._is_rudder_action_active

    @property
    def is_sail_action_active(self) -> bool:
        return self._is_sail_action_active

    @property
    def pub_period(self) -> float:
        return self.get_parameter("pub_period_sec").get_parameter_value().double_value

    @property
    def gps(self) -> Optional[GPS]:
        return self.__gps

    @property
    def gps_sub(self) -> Subscription:
        return self.__gps_sub

    @property
    def rudder_angle(self) -> Scalar:
        return self.__rudder_angle

    @property
    def sail_trim_tab_angle(self) -> Scalar:
        return self.__sail_trim_tab_angle


if __name__ == "__main__":
    main()
