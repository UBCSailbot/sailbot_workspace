#!/usr/bin/env python3

"""The ROS node for the physics engine."""

import json
import sys
from collections import deque
from typing import Optional, Union

import numpy as np
import rclpy
from custom_interfaces.action._sim_rudder_actuation import (
    SimRudderActuation_FeedbackMessage,
)
from custom_interfaces.action._sim_sail_trim_tab_actuation import (
    SimSailTrimTabActuation_FeedbackMessage,
)
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, Future
from rclpy.executors import Executor, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import CallbackGroup, MutuallyExclusiveCallbackGroup, Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from test_plans.test_plan import TestPlan

import boat_simulator.common.constants as Constants
import boat_simulator.common.utils as Utils
from boat_simulator.common.angle_conventions import (
    RudderAngle,
    TrimTabAngle,
    saturated_rudder_angle,
    saturated_trim_tab_angle,
)
from boat_simulator.common.conventions import NED, Velocity
from boat_simulator.common.generators import MVGaussianGenerator
from boat_simulator.common.sensors import SimGPS, SimWindSensor
from boat_simulator.common.types import Vec2
from boat_simulator.common.unit_conversions import ConversionFactors
from boat_simulator.nodes.physics_engine.fluid_generation import FluidGenerator
from boat_simulator.nodes.physics_engine.model import BoatState
from custom_interfaces.action import SimRudderActuation, SimSailTrimTabActuation
from custom_interfaces.msg import (
    GPS,
    DesiredHeading,
    HelperHeading,
    HelperLatLon,
    SailCmd,
    SimWorldState,
    WindSensor,
)

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
        wind_sensors_pub (Publisher): Publishes filtered wind data in a `WindSensor` message.
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
                ("physics_timestep_sec", rclpy.Parameter.Type.DOUBLE),
                ("logging_throttle_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("info_log_throttle_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("action_send_goal_timeout_sec", rclpy.Parameter.Type.DOUBLE),
                ("qos_depth", rclpy.Parameter.Type.INTEGER),
                ("test_plan", rclpy.Parameter.Type.STRING),
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
                (Constants.SIM_GPS_ORIGIN_LATITUDE_PARAM, rclpy.Parameter.Type.DOUBLE),
                (Constants.SIM_GPS_ORIGIN_LONGITUDE_PARAM, rclpy.Parameter.Type.DOUBLE),
            ],
        )

        # TODO Revisit this debug statement. It might get ugly for args with complicated structures
        all_parameters = self._parameters
        for name, parameter in all_parameters.items():
            value_str = str(parameter.value)
            self.get_logger().debug(f"Got parameter {name} with value {value_str}")

    def __init_private_attributes(self) -> None:
        """Initializes the private attributes of this class that are not set anywhere else during
        the initialization process.
        """
        self.__publish_counter = 0
        self.__rudder_angle: RudderAngle = RudderAngle(0.0)
        self.__sail_trim_tab_angle: TrimTabAngle = TrimTabAngle(0.0)
        self.__desired_heading: Union[DesiredHeading, None] = None

        self.test_plan = self.get_parameter("test_plan").get_parameter_value().string_value
        test_plan = TestPlan(self.test_plan)
        gps = test_plan.gps
        substeps = max(1, round(self.pub_period / self.physics_timestep))

        self.__reference_latlon: HelperLatLon = gps.lat_lon
        self.__boat_state = BoatState(
            self.pub_period / substeps, self.__reference_latlon, substeps
        )
        self.__sim_gps: Optional[SimGPS] = None

        # MVGaussianGenerator expects raw numpy arrays: a 1D mean vector and a 2D covariance
        # matrix (parsed from a string, since ROS parameters do not support 2D array types).
        wind_mean_kmph = np.array(
            self.get_parameter("wind_generation.mvgaussian_params.mean")
            .get_parameter_value()
            .double_array_value
        )
        wind_cov_kmph = np.array(
            json.loads(
                self.get_parameter("wind_generation.mvgaussian_params.cov")
                .get_parameter_value()
                .string_value
            )
        )
        self.__wind_generator = FluidGenerator(
            generator=MVGaussianGenerator(wind_mean_kmph, wind_cov_kmph)
        )
        self.__wind_sensor_readings_mps: deque[Vec2] = deque(maxlen=20)
        self.__sma_wind_mps: Vec2[Velocity, NED] = Vec2.from_xy(0.0, 0.0)
        current_mean_mps = np.array(
            self.get_parameter("current_generation.mvgaussian_params.mean")
            .get_parameter_value()
            .double_array_value
        )
        current_cov_mps = np.array(
            json.loads(
                self.get_parameter("current_generation.mvgaussian_params.cov")
                .get_parameter_value()
                .string_value
            )
        )
        self.__current_generator = FluidGenerator(
            generator=MVGaussianGenerator(current_mean_mps, current_cov_mps)
        )

        sim_wind_mps: Vec2[Velocity, NED] = self.__wind_generator.next()
        self.__sim_wind_sensor = SimWindSensor(sim_wind_mps, enable_noise=True)
        self.__latest_wind_sensor_reading_mps: Vec2[Velocity, NED] = self.__sim_wind_sensor.wind

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
        self.__rudder_pub = self.create_publisher(
            msg_type=HelperHeading,
            topic="rudder",
            qos_profile=10,
        )
        self.__wind_sensors_pub = self.create_publisher(
            msg_type=WindSensor,
            topic=Constants.PHYSICS_ENGINE_PUBLISHERS.FILTERED_WIND_SENSORS,
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

        # TODO Get updated boat state and publish (should this be separate from publishing?)
        self.__update_boat_state()
        self.__latest_wind_sensor_reading_mps = self.__sim_wind_sensor.wind

        gps_msg = self.__publish_gps()
        self.__publish_wind_sensors()
        self.__publish_kinematics(gps_msg)
        self.__publish_counter += 1

    def __update_boat_state(self):
        """
        Generates the next vectors for wind_generator and current_generator and updates the
        boat_state with the new wind and current vectors along with the rudder_angle and
        sail_trim_tab_angle.
        """
        true_wind_mps = self.__wind_generator.next()
        true_current_mps = self.__current_generator.next()
        self.__sim_wind_sensor.wind = true_wind_mps
        # The rudder/trim angles are already saturated value objects (saturation happens at the
        # actuation-feedback boundary), so they are forwarded to the boat model as-is.

        # Entry to the simulator
        self.__boat_state.step(
            true_wind_mps,
            true_current_mps,
            self.__rudder_angle,
            self.__sail_trim_tab_angle,
        )

    def __publish_gps(self) -> GPS:
        """Publishes mock GPS data.

        Returns:
            GPS: The published message, so `__publish_kinematics` can embed the same noisy
                reading instead of drawing a second, independent sample from `__sim_gps`.
        """
        lat_lon = self.__boat_state.global_lat_lon_position

        self.get_logger().info(f"Boat global position (lat_lon) to be published: {lat_lon}")
        speed_mps = self.__boat_state.linear_speed
        heading = self.__boat_state.true_bearing

        if self.__sim_gps:
            self.__sim_gps.lat_lon = lat_lon
            self.__sim_gps.speed = speed_mps
            self.__sim_gps.heading = heading
        else:
            self.__sim_gps = SimGPS(
                lat_lon=lat_lon, speed=speed_mps, heading=heading, enable_noise=True
            )

        gps_msg = self.__convert_sim_data_to_gps_msg()
        rudder_msg = HelperHeading(heading=heading.degrees)

        self.gps_pub.publish(gps_msg)

        self.rudder_pub.publish(rudder_msg)

        self.get_logger().debug(
            f"Publishing to {self.gps_pub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )
        return gps_msg

    def __convert_sim_data_to_gps_msg(self) -> GPS:
        """Builds a GPS message from the current sensor readings (noisy if noise is enabled).

        Conversions to the ROS message conventions happen at this I/O boundary: speed from
        m/s to km/h, and heading from the simulator's CCW-straight convention to the GPS
        CW-north convention in degrees.
        """
        if self.__sim_gps is None:
            self.get_logger().warn("sim GPS sensor has not been initialized")
            return GPS()

        msg = GPS()
        lat, lon = self.__sim_gps.lat_lon
        msg.lat_lon.latitude = float(lat)
        msg.lat_lon.longitude = float(lon)
        msg.speed.speed = float(
            ConversionFactors.mPs_to_kmPh.value.forward_convert(self.__sim_gps.speed)
        )
        msg.heading.heading = float(
            Utils.ccw_straight_to_cw_north_deg(self.__sim_gps.heading.degrees)
        )
        return msg

    def __publish_wind_sensors(self):
        """Publishes filtered wind sensor data using a simple moving average."""
        raw_wind_mps = self.__latest_wind_sensor_reading_mps
        k = self.__wind_sensor_readings_mps.maxlen
        if k is None:
            raise RuntimeError("wind sensor reading window must have a fixed size")
        n = len(self.__wind_sensor_readings_mps)

        if n < k:
            # Queue not yet full: approximate with fixed-window weighted update
            self.__sma_wind_mps = Vec2(
                (self.__sma_wind_mps.data * (k - 1) + raw_wind_mps.data) / k
            )
        else:
            # Queue full: incremental SMA — add new, subtract oldest [Units in meters per second]
            oldest_mps = self.__wind_sensor_readings_mps[0]
            self.__sma_wind_mps = Vec2(
                self.__sma_wind_mps.data + (raw_wind_mps.data - oldest_mps.data) / k
            )

        self.__wind_sensor_readings_mps.append(raw_wind_mps)

        msg = WindSensor()

        # Convert sim wind sensor data from SI to ROS message units
        mps_to_kmph = ConversionFactors.mPs_to_kmPh.value.forward_convert
        sma_wind_kmph = Vec2(mps_to_kmph(self.__sma_wind_mps.data))
        msg.speed.speed = float(Utils.get_wind_speed(sma_wind_kmph))
        msg.direction = int(Utils.get_wind_direction(sma_wind_kmph))

        self.wind_sensors_pub.publish(msg)
        self.get_logger().debug(
            f"Publishing to {self.wind_sensors_pub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )

    def __publish_kinematics(self, gps_msg: GPS):
        """Publishes the kinematics data of the simulated boat.

        Args:
            gps_msg (GPS): The message already published by `__publish_gps` this tick, reused
                here as `SimWorldState.global_gps` so both topics report the same noisy GPS
                reading instead of two independently-sampled ones.
        """
        msg = self.__convert_sim_data_to_kinematics_msg(gps_msg)

        self.kinematics_pub.publish(msg)

        self.get_logger().info(
            f"Publishing to {self.kinematics_pub.topic}",
            throttle_duration_sec=self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value,
        )

    def __convert_sim_data_to_kinematics_msg(self, gps_msg: GPS) -> SimWorldState:
        """Builds a SimWorldState message from the current sim state.

        Unit/frame conversions to the message conventions happen here, at the I/O boundary:
        position from m to km, linear velocities from m/s to km/h, and the boat orientation
        from Euler angles to a quaternion.

        Args:
            gps_msg (GPS): The GPS reading to embed as `global_gps`, reused from
                `__publish_gps` rather than resampled here.
        """

        mps_to_kmph = ConversionFactors.mPs_to_kmPh.value.forward_convert
        m_to_km = ConversionFactors.m_to_km.value.forward_convert

        msg = SimWorldState()
        msg.global_gps = gps_msg

        pos_m = self.__boat_state.pose
        msg.global_pose.position.x = float(m_to_km(pos_m.x))
        msg.global_pose.position.y = float(m_to_km(pos_m.y))
        msg.global_pose.position.z = 0.0

        quat = Utils.euler_zyx_to_quaternion(roll_rad=pos_m.p, pitch_rad=0.0, yaw_rad=pos_m.r)
        msg.global_pose.orientation.x = float(quat[0])
        msg.global_pose.orientation.y = float(quat[1])
        msg.global_pose.orientation.z = float(quat[2])
        msg.global_pose.orientation.w = float(quat[3])

        msg.wind_velocity.x = float(mps_to_kmph(self.__wind_generator.velocity.x))
        msg.wind_velocity.y = float(mps_to_kmph(self.__wind_generator.velocity.y))
        msg.wind_velocity.z = 0.0

        msg.current_velocity.x = float(mps_to_kmph(self.__current_generator.velocity.x))
        msg.current_velocity.y = float(mps_to_kmph(self.__current_generator.velocity.y))
        msg.current_velocity.z = 0.0

        sec, nanosec = divmod(self.pub_period * self.publish_counter, 1)
        msg.header.stamp.sec = int(sec)
        msg.header.stamp.nanosec = int(nanosec * 1e9)
        msg.header.frame_id = str(self.publish_counter)

        return msg

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
        self.__sail_trim_tab_angle = saturated_trim_tab_angle(
            Utils.degrees_to_rad(msg.trim_tab_angle_degrees)
        )

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
            + f"{result.remaining_angular_distance:.2f} degrees and final "
            + f"rudder angle of {self.rudder_angle.degrees:.2f} degrees"
        )

    def __rudder_action_feedback_callback(self, feedback_msg: SimRudderActuation_FeedbackMessage):
        """Updates the rudder angle as the rudder action routine executes. As the action routine
        publishes feedback, this function is executed.

        Args:
            feedback_msg (SimRudderActuation_FeedbackMessage): The feedback message.
        """
        self.__rudder_angle = saturated_rudder_angle(
            Utils.degrees_to_rad(feedback_msg.feedback.rudder_angle)
        )
        self.get_logger().info(
            f"Received rudder angle of {self.rudder_angle.degrees:.2f} degrees from action "
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
        goal_msg.desired_angular_position = self.sail_trim_tab_angle.degrees

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
            + f"trim tab angle of {self.sail_trim_tab_angle.degrees:.2f} degrees"
        )

    def __sail_action_feedback_callback(
        self, feedback_msg: SimSailTrimTabActuation_FeedbackMessage
    ):
        """Updates the sail trim tab angle as the sail action routine executes. As the action
        routine publishes feedback, this function is executed.

        Args:
            feedback_msg (SimSailTrimTabActuation_FeedbackMessage): The feedback message.
        """
        self.__sail_trim_tab_angle = saturated_trim_tab_angle(
            Utils.degrees_to_rad(feedback_msg.feedback.current_angular_position)
        )
        self.get_logger().info(
            f"Received sail trim tab angle of {self.sail_trim_tab_angle.degrees:.2f} degrees "
            + f"from action {self.sail_actuation_action_client._action_name}",
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
    def physics_timestep(self) -> float:
        return self.get_parameter("physics_timestep_sec").get_parameter_value().double_value

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
    def rudder_pub(self) -> Publisher:
        return self.__rudder_pub

    @property
    def desired_heading(self) -> Optional[DesiredHeading]:
        return self.__desired_heading

    @property
    def desired_heading_sub(self) -> Subscription:
        return self.__desired_heading_sub

    @property
    def rudder_angle(self) -> RudderAngle:
        return self.__rudder_angle

    @property
    def sail_trim_tab_angle(self) -> TrimTabAngle:
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
