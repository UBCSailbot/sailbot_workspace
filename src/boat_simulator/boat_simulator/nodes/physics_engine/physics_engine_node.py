#!/usr/bin/env python3

"""The ROS node for the physics engine."""

import json
import sys
from typing import Optional

import numpy as np
import rclpy
import rclpy.utilities
from custom_interfaces.action import SimRudderActuation, SimSailTrimTabActuation
from custom_interfaces.action._sim_rudder_actuation import (
    SimRudderActuation_FeedbackMessage,
)
from custom_interfaces.action._sim_sail_trim_tab_actuation import (
    SimSailTrimTabActuation_FeedbackMessage,
)
from custom_interfaces.msg import (
    GPS,
    DesiredHeading,
    SailCmd,
    SimWorldState,
    WindSensor,
    WindSensors,
)
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, Future
from rclpy.executors import Executor, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import CallbackGroup, MutuallyExclusiveCallbackGroup, Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

import boat_simulator.common.constants as Constants
from boat_simulator.common.generators import MVGaussianGenerator
from boat_simulator.common.sensors import SimWindSensor
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.fluid_generation import FluidGenerator
from boat_simulator.nodes.physics_engine.model import BoatState

from .decorators import require_all_subs_active


def main(args=None):
    rclpy.init(args=args)
    multithreading_enabled = is_multithreading_enabled()
    node = PhysicsEngineNode(multithreading_enabled=multithreading_enabled)
    executor = get_executor(is_multithreading_enabled=multithreading_enabled)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


def is_multithreading_enabled() -> bool:
    try:
        is_multithreading_enabled_index = sys.argv.index(Constants.MULTITHREADING_CLI_ARG_NAME) + 1
        is_multithreading_enabled = sys.argv[is_multithreading_enabled_index] == "true"
    except ValueError:
        is_multithreading_enabled = False
    return is_multithreading_enabled


def get_executor(is_multithreading_enabled: bool) -> Executor:
    if is_multithreading_enabled:
        return MultiThreadedExecutor()
    else:
        return SingleThreadedExecutor()


class PhysicsEngineNode(Node):
    """Stores, updates, and maintains the state of the physics model of the boat simulator.

    Subscriptions:
        desired_heading_sub (Subscription): Subscribes to a `DesiredHeading` message.

    Publishers:
        gps_pub (Publisher): Publishes GPS data in a `GPS` message.
        wind_sensors_pub (Publisher): Publishes wind sensor data in a `WindSensors` message.
        kinematics_pub (Publisher): Publishes kinematics data in a `SimWorldState` message.

    Action Clients:
        rudder_actuation_action_client (ActionClient): Requests rudder actuations.
        sail_actuation_action_client (ActionClient): Requests sail trim tab actuations.
    """

    def __init__(self, multithreading_enabled: bool):
        """Initializes an instance of this class.

        Args:
            multithreading_enabled (bool): True if this node uses a multithreaded executor, and
            false for a single threaded executor.
        """
        super().__init__(node_name="physics_engine_node")
        self.__is_multithreading_enabled = multithreading_enabled

        self.get_logger().debug("Initializing node...")
        self.__declare_ros_parameters()
        self.__init_private_attributes()
        self.__init_callback_groups()
        self.__init_subscriptions()
        self.__init_publishers()
        self.__init_action_clients()
        self.__init_timer_callbacks()
        self.get_logger().debug("Node initialization complete. Starting execution...")

    # INITIALIZATION HELPERS
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
                ("action_send_goal_timeout_sec", rclpy.Parameter.Type.DOUBLE),
                ("qos_depth", rclpy.Parameter.Type.INTEGER),
                ("rudder.actuation_request_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("wingsail.actuation_request_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("wind_sensor.generator_type", rclpy.Parameter.Type.STRING),
                ("wind_sensor.gaussian_params.mean", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("wind_sensor.gaussian_params.std_dev", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("wind_sensor.gaussian_params.corr_xy", rclpy.Parameter.Type.DOUBLE),
                ("wind_sensor.constant_params.value", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("wind_generation.mvgaussian_params.mean", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("wind_generation.mvgaussian_params.cov", rclpy.Parameter.Type.STRING),
                ("current_generation.mvgaussian_params.mean", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("current_generation.mvgaussian_params.cov", rclpy.Parameter.Type.STRING),
            ],
        )

        # TODO Revisit this debug statement. It might get ugly for args with complicated structures
        all_parameters = self._parameters
        for name, parameter in all_parameters.items():
            value_str = str(parameter.value)
            self.get_logger().debug(f"Got parameter {name} with value {value_str}")

    def __init_private_attributes(self):
        """Initializes the private attributes of this class that are not set anywhere else during
        the initialization process.
        """
        self.__publish_counter = 0
        self.__rudder_angle = 0
        self.__sail_trim_tab_angle = 0
        self.__desired_heading = None
        self.__boat_state = BoatState(self.pub_period)

        wind_mean = np.array(
            self.get_parameter("wind_generation.mvgaussian_params.mean")
            .get_parameter_value()
            .double_array_value
        )
        # Parse the covariance matrix from a string into a 2D array, as ROS parameters do not
        # support native 2D array types.
        wind_cov = np.array(
            json.loads(
                self.get_parameter("wind_generation.mvgaussian_params.cov")
                .get_parameter_value()
                .string_value
            )
        )
        self.__wind_generator = FluidGenerator(generator=MVGaussianGenerator(wind_mean, wind_cov))

        current_mean = np.array(
            self.get_parameter("current_generation.mvgaussian_params.mean")
            .get_parameter_value()
            .double_array_value
        )
        # Parse the covariance matrix from a string into a 2D array, as ROS parameters do not
        # support native 2D array types.
        current_cov = np.array(
            json.loads(
                self.get_parameter("current_generation.mvgaussian_params.cov")
                .get_parameter_value()
                .string_value
            )
        )
        self.__current_generator = FluidGenerator(
            generator=MVGaussianGenerator(current_mean, current_cov)
        )

        # No delay in this instance
        sim_wind = self.__wind_generator.next()
        self.__sim_wind_sensor = SimWindSensor(sim_wind, enable_noise=True)

    def __init_callback_groups(self):
        """Initializes the callback groups. Whether multithreading is enabled or not will affect
        how callbacks are executed.

        If multithreading is enabled: Callbacks belonging to different callback groups may execute
        in parallel to each other.

        If multithreading is disabled: All callbacks are assigned to the same default callback
        group, and only one callback may execute at a time in a single-threaded manner
        (the ROS2 default).

        Learn more about executors and callback groups here:
        https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html#executors
        """
        # TODO Consider if data to each topic should be published in parallel or synchronously
        if self.is_multithreading_enabled:
            self.get_logger().debug(
                "Multithreading enabled. Initializing multiple callback groups"
            )
            self.__pub_callback_group = MutuallyExclusiveCallbackGroup()
            self.__sub_callback_group = MutuallyExclusiveCallbackGroup()
            self.__rudder_action_callback_group = MutuallyExclusiveCallbackGroup()
            self.__sail_action_callback_group = MutuallyExclusiveCallbackGroup()
        else:
            self.get_logger().debug(
                "Multithreading disabled. Assigning all callbacks to the default callback group"
            )
            self.__pub_callback_group = self.default_callback_group
            self.__sub_callback_group = self.default_callback_group
            self.__rudder_action_callback_group = self.default_callback_group
            self.__sail_action_callback_group = self.default_callback_group

    def __init_subscriptions(self):
        """Initializes the subscriptions of this node. Subscriptions pull data from other ROS
        topics for further usage in this node. Data is pulled from subscriptions periodically via
        callbacks, which are registered upon subscription initialization.
        """
        # TODO Subscribe to CAN/Sim interface output to replace the current subscriptions
        self.get_logger().debug("Initializing subscriptions...")
        self.__desired_heading_sub = self.create_subscription(
            msg_type=DesiredHeading,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.DESIRED_HEADING,
            callback=self.__desired_heading_sub_callback,
            qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
            callback_group=self.sub_callback_group,
        )
        self.__sail_trim_tab_angle_sub = self.create_subscription(
            msg_type=SailCmd,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.SAIL_TRIM_TAB_ANGLE,
            callback=self.__sail_trim_tab_angle_sub_callback,
            qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
            callback_group=self.sub_callback_group,
        )

    def __init_publishers(self):
        """Initializes the publishers of this node. Publishers update ROS topics so that other ROS
        nodes in the system can utilize the data produced by this node.
        """
        self.get_logger().debug("Initializing publishers...")
        self.__gps_pub = self.create_publisher(
            msg_type=GPS,
            topic=Constants.PHYSICS_ENGINE_PUBLISHERS.GPS,
            qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
        )
        self.__wind_sensors_pub = self.create_publisher(
            msg_type=WindSensors,
            topic=Constants.PHYSICS_ENGINE_PUBLISHERS.WIND_SENSORS,
            qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
        )
        self.__kinematics_pub = self.create_publisher(
            msg_type=SimWorldState,
            topic=Constants.PHYSICS_ENGINE_PUBLISHERS.KINEMATICS,
            qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
        )

    def __init_action_clients(self):
        """Initializes the action clients of this node. Action clients initiate requests to the
        action server to perform longer running tasks like rudder actuation, sail actuation, etc.
        while also periodically receiving feedback from the action server as it completes its task.
        """
        self.get_logger().debug("Initializing action clients...")
        self.__rudder_actuation_action_client = ActionClient(
            node=self,
            action_type=SimRudderActuation,
            action_name=Constants.ACTION_NAMES.RUDDER_ACTUATION,
            callback_group=self.rudder_action_callback_group,
        )
        self.__sail_actuation_action_client = ActionClient(
            node=self,
            action_type=SimSailTrimTabActuation,
            action_name=Constants.ACTION_NAMES.SAIL_ACTUATION,
            callback_group=self.sail_action_callback_group,
        )

    def __init_timer_callbacks(self):
        """Initializes timer callbacks of this node. Timer callbacks are executed periodically with
        a specified execution frequency.
        """
        self.get_logger().debug("Initializing timer callbacks...")

        # Publishing data to ROS topics
        self.create_timer(
            timer_period_sec=self.pub_period,
            callback=self.__publish,
            callback_group=self.pub_callback_group,
        )

        # Requesting a rudder actuation
        self.create_timer(
            timer_period_sec=self.get_parameter("rudder.actuation_request_period_sec")
            .get_parameter_value()
            .double_value,
            callback=self.__rudder_action_send_goal,
            callback_group=self.rudder_action_callback_group,
        )

        # Requesting a sail actuation
        self.create_timer(
            timer_period_sec=self.get_parameter("wingsail.actuation_request_period_sec")
            .get_parameter_value()
            .double_value,
            callback=self.__sail_action_send_goal,
            callback_group=self.sail_action_callback_group,
        )

    # PUBLISHER CALLBACKS
    def __publish(self):
        """Synchronously publishes data to all publishers at once."""
        self.__update_boat_state()
        # TODO Get updated boat state and publish (should this be separate from publishing?)
        # TODO Get wind sensor data and publish (should this be separate from publishing?)
        self.__publish_gps()
        self.__publish_wind_sensors()
        self.__publish_kinematics()
        self.__publish_counter += 1

    def __update_boat_state(self):
        """
        Generates the next vectors for wind_generator and current_generator and updates the
        boat_state with the new wind and current vectors along with the rudder_angle and
        sail_trim_tab_angle.
        """
        # Wind parameter in line below introduces noise
        self.__sim_wind_sensor.wind = self.__wind_generator.next()
        self.__boat_state.step(
            self.__sim_wind_sensor.wind,
            self.__current_generator.next(),
            self.__rudder_angle,
            self.__sail_trim_tab_angle,
        )

    def __publish_gps(self):
        """Publishes mock GPS data."""
        # TODO Update to publish real data
        msg = GPS()
        msg.lat_lon.latitude = 0.0
        msg.lat_lon.longitude = 0.0
        msg.speed.speed = 0.0
        msg.heading.heading = 0.0

        self.gps_pub.publish(msg)
        self.get_logger().info(
            f"Publishing to {self.gps_pub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )

    def __publish_wind_sensors(self):
        """Publishes mock wind sensor data."""
        # TODO Update to publish real data
        windSensor1 = WindSensor()
        windSensor1.speed.speed = 0.0
        windSensor1.direction = 0

        windSensor2 = WindSensor()
        windSensor2.speed.speed = 0.0
        windSensor2.direction = 0

        msg = WindSensors()
        msg.wind_sensors = [windSensor1, windSensor2]

        self.wind_sensors_pub.publish(msg)
        self.get_logger().info(
            f"Publishing to {self.wind_sensors_pub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )

    def __publish_kinematics(self):
        """Publishes the kinematics data of the simulated boat."""
        # TODO Update to publish real data
        msg = SimWorldState()

        msg.global_gps.lat_lon.latitude = 0.0
        msg.global_gps.lat_lon.longitude = 0.0
        msg.global_gps.speed.speed = 0.0
        msg.global_gps.heading.heading = 0.0

        # had to make elements into float or else I would get issues
        msg.global_pose.position.x = float(self.__boat_state.global_position[0])
        msg.global_pose.position.y = float(self.__boat_state.global_position[1])
        msg.global_pose.position.z = float(self.__boat_state.global_position[2])
        msg.global_pose.orientation.x = float(self.__boat_state.angular_position[0])
        msg.global_pose.orientation.y = float(self.__boat_state.angular_position[1])
        msg.global_pose.orientation.z = float(self.__boat_state.angular_position[2])
        msg.global_pose.orientation.w = 1.0

        msg.wind_velocity.x = self.__wind_generator.velocity[0]
        msg.wind_velocity.y = self.__wind_generator.velocity[1]
        msg.wind_velocity.z = self.__wind_generator.velocity[2]

        msg.current_velocity.x = self.__current_generator.velocity[0]
        msg.current_velocity.y = self.__current_generator.velocity[1]
        msg.current_velocity.z = self.__current_generator.velocity[2]

        sec, nanosec = divmod(self.pub_period * self.publish_counter, 1)
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nanosec * 1e9)
        msg.header.frame_id = str(self.publish_counter)

        self.kinematics_pub.publish(msg)

        self.get_logger().info(
            f"Publishing to {self.kinematics_pub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )

    # SUBSCRIPTION CALLBACKS
    def __desired_heading_sub_callback(self, msg: DesiredHeading):
        """Stores the latest desired heading data.

        Args:
            msg (DesiredHeading): The desired heading data from local pathfinding.
        """
        self.get_logger().info(
            f"Received data from {self.desired_heading_sub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )
        self.__desired_heading = msg

    def __sail_trim_tab_angle_sub_callback(self, msg: SailCmd):
        """Stores the latest trim tab angle from the controller.

        Args:
            msg (SailCmd): The desired trim tab angle.
        """
        self.get_logger().info(
            f"Received data from {self.sail_trim_tab_angle_sub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )
        self.__sail_trim_tab_angle = msg.trim_tab_angle_degrees

    # RUDDER ACTUATION ACTION CLIENT CALLBACKS
    @require_all_subs_active
    def __rudder_action_send_goal(self):
        """Asynchronously sends a goal request to the rudder actuation action server
        and registers a callback to execute when the server routine is complete.

        All subscriptions of this node must be active for a successful action execution.
        """
        self.get_logger().debug("Initiating goal request for rudder actuation action")

        # Create the goal message
        goal_msg = SimRudderActuation.Goal()
        goal_msg.desired_heading = self.desired_heading

        action_send_goal_timeout_sec = (
            self.get_parameter("action_send_goal_timeout_sec").get_parameter_value().double_value
        )

        # Wait for the action server to be ready (the low-level control node)
        is_request_timed_out = not self.rudder_actuation_action_client.wait_for_server(
            timeout_sec=action_send_goal_timeout_sec
        )

        if is_request_timed_out:
            self.get_logger().warn(
                "Rudder actuation action goal request timed out after "
                + f"{action_send_goal_timeout_sec} seconds. Aborting..."
            )
        else:
            send_goal_future = self.rudder_actuation_action_client.send_goal_async(
                goal=goal_msg, feedback_callback=self.__rudder_action_feedback_callback
            )
            send_goal_future.add_done_callback(self.__rudder_action_goal_response_callback)
            self.get_logger().debug("Completed goal request for rudder actuation action")

    def __rudder_action_goal_response_callback(self, future: Future):
        """Prepares the execution process after the rudder action routine is complete. This
        function executes when the rudder actuation goal request has been acknowledged.

        Args:
            future (Future): The outcome of the goal request in the future.
        """
        goal_handle: Optional[ClientGoalHandle] = future.result()
        if (not goal_handle) or (not goal_handle.accepted):
            self.get_logger().warn("Attempted to send rudder actuation goal, but it was rejected")
            return
        self.get_logger().debug("Rudder actuation goal was accepted. Beginning rudder actuation")
        rudder_get_result_future = goal_handle.get_result_async()
        rudder_get_result_future.add_done_callback(self.__rudder_action_get_result_callback)

    def __rudder_action_get_result_callback(self, future: Future):
        """The execution process after the rudder action routine is complete.

        Args:
            future (Future): The outcome of the rudder action routine in the future.
        """
        result = future.result().result
        self.get_logger().debug(
            "Rudder actuation action finished with a heading residual of "
            + f"{result.remaining_angular_distance:.2f} rad and final "
            + f"rudder angle of {self.rudder_angle:.2f} rad"
        )

    def __rudder_action_feedback_callback(self, feedback_msg: SimRudderActuation_FeedbackMessage):
        """Updates the rudder angle as the rudder action routine executes. As the action routine
        publishes feedback, this function is executed.

        Args:
            feedback_msg (SimRudderActuation_FeedbackMessage): The feedback message.
        """
        self.__rudder_angle = feedback_msg.feedback.rudder_angle
        self.get_logger().info(
            f"Received rudder angle of {self.rudder_angle:.2f} rad from action "
            + f"{self.rudder_actuation_action_client._action_name}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )

    # SAIL ACTUATION ACTION CLIENT CALLBACKS
    @require_all_subs_active
    def __sail_action_send_goal(self):
        """Asynchronously sends a goal request to the sail actuation action server
        and registers a callback to execute when the server routine is complete.

        All subscriptions of this node must be active for a successful action execution.
        """
        self.get_logger().debug("Initiating goal request for sail actuation action")

        # Create the goal message
        goal_msg = SimSailTrimTabActuation.Goal()
        goal_msg.desired_angular_position = self.sail_trim_tab_angle

        action_send_goal_timeout_sec = (
            self.get_parameter("action_send_goal_timeout_sec").get_parameter_value().double_value
        )

        # Wait for the action server to be ready (the low-level control node)
        is_request_timed_out = not self.sail_actuation_action_client.wait_for_server(
            timeout_sec=action_send_goal_timeout_sec
        )

        if is_request_timed_out:
            self.get_logger().warn(
                "Sail actuation action goal request timed out after "
                + f"{action_send_goal_timeout_sec} seconds. Aborting..."
            )
        else:
            send_goal_future = self.sail_actuation_action_client.send_goal_async(
                goal=goal_msg, feedback_callback=self.__sail_action_feedback_callback
            )
            send_goal_future.add_done_callback(self.__sail_action_goal_response_callback)
            self.get_logger().debug("Completed goal request for sail actuation action")

    def __sail_action_goal_response_callback(self, future: Future):
        """Prepares the execution process after the sail action routine is complete. This
        function executes when the sail actuation goal request has been acknowledged.

        Args:
            future (Future): The outcome of the goal request in the future.
        """
        goal_handle: Optional[ClientGoalHandle] = future.result()
        if (not goal_handle) or (not goal_handle.accepted):
            self.get_logger().warn("Attempted to send sail actuation goal, but it was rejected")
            return
        self.get_logger().debug("Sail actuation goal was accepted. Beginning sail actuation")
        sail_get_result_future = goal_handle.get_result_async()
        sail_get_result_future.add_done_callback(self.__sail_action_get_result_callback)

    def __sail_action_get_result_callback(self, future: Future):
        """The execution process after the sail action routine is complete.

        Args:
            future (Future): The outcome of the sail action routine in the future.
        """
        result = future.result().result
        self.get_logger().debug(
            "Sail actuation action finished with an angular residual of "
            + f"{result.remaining_angular_distance:.2f} rad and final "
            + f"trim tab angle of {self.sail_trim_tab_angle:.2f} rad"
        )

    def __sail_action_feedback_callback(
        self, feedback_msg: SimSailTrimTabActuation_FeedbackMessage
    ):
        """Updates the sail trim tab angle as the sail action routine executes. As the action
        routine publishes feedback, this function is executed.

        Args:
            feedback_msg (SimSailTrimTabActuation_FeedbackMessage): The feedback message.
        """
        self.__sail_trim_tab_angle = feedback_msg.feedback.current_angular_position
        self.get_logger().info(
            f"Received sail trim tab angle of {self.sail_trim_tab_angle:.2f} rad from action "
            + f"{self.sail_actuation_action_client._action_name}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )

    # CLASS PROPERTY PUBLIC GETTERS
    @property
    def is_multithreading_enabled(self) -> bool:
        return self.__is_multithreading_enabled

    @property
    def pub_callback_group(self) -> CallbackGroup:
        return self.__pub_callback_group

    @property
    def sub_callback_group(self) -> CallbackGroup:
        return self.__sub_callback_group

    @property
    def rudder_action_callback_group(self) -> CallbackGroup:
        return self.__rudder_action_callback_group

    @property
    def sail_action_callback_group(self) -> CallbackGroup:
        return self.__sail_action_callback_group

    @property
    def pub_period(self) -> float:
        return self.get_parameter("pub_period_sec").get_parameter_value().double_value

    @property
    def publish_counter(self) -> int:
        return self.__publish_counter

    @property
    def gps_pub(self) -> Publisher:
        return self.__gps_pub

    @property
    def wind_sensors_pub(self) -> Publisher:
        return self.__wind_sensors_pub

    @property
    def kinematics_pub(self) -> Publisher:
        return self.__kinematics_pub

    @property
    def desired_heading(self) -> Optional[DesiredHeading]:
        return self.__desired_heading

    @property
    def desired_heading_sub(self) -> Subscription:
        return self.__desired_heading_sub

    @property
    def rudder_angle(self) -> Scalar:
        return self.__rudder_angle

    @property
    def sail_trim_tab_angle(self) -> Scalar:
        return self.__sail_trim_tab_angle

    @property
    def sail_trim_tab_angle_sub(self) -> Subscription:
        return self.__sail_trim_tab_angle_sub

    @property
    def rudder_actuation_action_client(self) -> ActionClient:
        return self.__rudder_actuation_action_client

    @property
    def sail_actuation_action_client(self) -> ActionClient:
        return self.__sail_actuation_action_client


if __name__ == "__main__":
    main()
