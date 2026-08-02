#!/usr/bin/env python3

"""ROS node that adapts a Gazebo (gz-sim) simulation to the boat simulator's ROS contract.

This node is a drop-in replacement for boat_simulator's `physics_engine_node`: it exposes the
same publishers, subscriptions, and action clients, but sources the boat state from Gazebo
odometry (via ros_gz_bridge) instead of integrating the equations of motion itself, and it
forwards actuation feedback to Gazebo joint controllers.
"""

import math
from typing import List, Optional

import rclpy
from custom_interfaces.action._sim_rudder_actuation import (
    SimRudderActuation_FeedbackMessage,
)
from custom_interfaces.action._sim_sail_trim_tab_actuation import (
    SimSailTrimTabActuation_FeedbackMessage,
)
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, Future
from rclpy.node import Node
from std_msgs.msg import Float64

import boat_simulator.common.constants as Constants
import boat_simulator.common.utils as Utils
from boat_simulator.common.geo_conversions import local_position_to_gps_lat_lon
from boat_simulator.common.unit_conversions import ConversionFactors
from custom_interfaces.action import SimRudderActuation, SimSailTrimTabActuation
from custom_interfaces.msg import (
    GPS,
    DesiredHeading,
    HelperHeading,
    SailCmd,
    SimWorldState,
    WindSensor,
)

# ROS-side names of the ros_gz_bridge topics. Must stay in sync with the remappings in
# BRIDGED_TOPICS in launch/gazebo_launch.py.
GAZEBO_ODOMETRY_TOPIC = "/gazebo/odometry"
GAZEBO_RUDDER_CMD_TOPIC = "/gazebo/rudder_angle_cmd"
GAZEBO_SAIL_TRIM_TAB_CMD_TOPIC = "/gazebo/sail_trim_tab_angle_cmd"


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPhysicsBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()


class GazeboPhysicsBridgeNode(Node):
    """Bridges Gazebo simulation state into the boat simulator's ROS contract.

    Subscriptions:
        desired_heading_sub (Subscription): Subscribes to a `DesiredHeading` message.
        sail_trim_tab_angle_sub (Subscription): Subscribes to a `SailCmd` message.
        gazebo_odometry_sub (Subscription): Subscribes to the bridged Gazebo `Odometry`.

    Publishers:
        gps_pub (Publisher): Publishes GPS data in a `GPS` message.
        wind_sensors_pub (Publisher): Publishes apparent wind data in a `WindSensor` message.
        kinematics_pub (Publisher): Publishes kinematics data in a `SimWorldState` message.
        rudder_pub (Publisher): Publishes the boat heading in a `HelperHeading` message.
        gazebo_rudder_cmd_pub (Publisher): Publishes rudder joint commands toward Gazebo.
        gazebo_sail_cmd_pub (Publisher): Publishes trim tab joint commands toward Gazebo.

    Action Clients:
        rudder_actuation_action_client (ActionClient): Requests rudder actuations.
        sail_actuation_action_client (ActionClient): Requests sail trim tab actuations.
    """

    def __init__(self):
        super().__init__(node_name="gazebo_physics_bridge_node")
        self.get_logger().debug("Initializing node...")
        self.__declare_ros_parameters()
        self.__init_private_attributes()
        self.__init_subscriptions()
        self.__init_publishers()
        self.__init_action_clients()
        self.__init_timer_callbacks()
        self.get_logger().debug("Node initialization complete. Starting execution...")

    # INITIALIZATION HELPERS
    def __declare_ros_parameters(self):
        """Declares ROS parameters from the global configuration file used by this node."""
        self.get_logger().debug("Declaring ROS parameters...")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("info_log_throttle_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("action_send_goal_timeout_sec", rclpy.Parameter.Type.DOUBLE),
                ("qos_depth", rclpy.Parameter.Type.INTEGER),
                ("rudder.actuation_request_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("wingsail.actuation_request_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("true_wind_enu_mps", rclpy.Parameter.Type.DOUBLE_ARRAY),
                (Constants.SIM_GPS_ORIGIN_LATITUDE_PARAM, rclpy.Parameter.Type.DOUBLE),
                (Constants.SIM_GPS_ORIGIN_LONGITUDE_PARAM, rclpy.Parameter.Type.DOUBLE),
            ],
        )

    def __init_private_attributes(self):
        """Initializes private attributes not set anywhere else during initialization."""
        self.__publish_counter = 0
        self.__latest_odometry: Optional[Odometry] = None
        self.__desired_heading: Optional[DesiredHeading] = None
        self.__rudder_angle_deg = 0.0
        self.__sail_trim_tab_angle_deg = 0.0

        self.__gps_origin_lat = (
            self.get_parameter(Constants.SIM_GPS_ORIGIN_LATITUDE_PARAM)
            .get_parameter_value()
            .double_value
        )
        self.__gps_origin_lon = (
            self.get_parameter(Constants.SIM_GPS_ORIGIN_LONGITUDE_PARAM)
            .get_parameter_value()
            .double_value
        )

        true_wind = (
            self.get_parameter("true_wind_enu_mps").get_parameter_value().double_array_value
        )
        if len(true_wind) < 2 or not all(math.isfinite(component) for component in true_wind):
            raise ValueError("true_wind_enu_mps must contain finite [east, north] components")
        self.__true_wind_enu_mps = (float(true_wind[0]), float(true_wind[1]))

    def __init_subscriptions(self):
        """Initializes the subscriptions of this node."""
        self.get_logger().debug("Initializing subscriptions...")
        qos_depth = self.get_parameter("qos_depth").get_parameter_value().integer_value
        self.__desired_heading_sub = self.create_subscription(
            msg_type=DesiredHeading,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.DESIRED_HEADING,
            callback=self.__desired_heading_sub_callback,
            qos_profile=qos_depth,
        )
        self.__sail_trim_tab_angle_sub = self.create_subscription(
            msg_type=SailCmd,
            topic=Constants.PHYSICS_ENGINE_SUBSCRIPTIONS.SAIL_TRIM_TAB_ANGLE,
            callback=self.__sail_trim_tab_angle_sub_callback,
            qos_profile=qos_depth,
        )
        self.__gazebo_odometry_sub = self.create_subscription(
            msg_type=Odometry,
            topic=GAZEBO_ODOMETRY_TOPIC,
            callback=self.__gazebo_odometry_sub_callback,
            qos_profile=qos_depth,
        )

    def __init_publishers(self):
        """Initializes the publishers of this node."""
        self.get_logger().debug("Initializing publishers...")
        qos_depth = self.get_parameter("qos_depth").get_parameter_value().integer_value
        self.__gps_pub = self.create_publisher(
            msg_type=GPS,
            topic=Constants.PHYSICS_ENGINE_PUBLISHERS.GPS,
            qos_profile=qos_depth,
        )
        self.__rudder_pub = self.create_publisher(
            msg_type=HelperHeading,
            topic="rudder",
            qos_profile=10,
        )
        self.__wind_sensors_pub = self.create_publisher(
            msg_type=WindSensor,
            topic=Constants.PHYSICS_ENGINE_PUBLISHERS.FILTERED_WIND_SENSORS,
            qos_profile=qos_depth,
        )
        self.__kinematics_pub = self.create_publisher(
            msg_type=SimWorldState,
            topic=Constants.PHYSICS_ENGINE_PUBLISHERS.KINEMATICS,
            qos_profile=qos_depth,
        )
        self.__gazebo_rudder_cmd_pub = self.create_publisher(
            msg_type=Float64,
            topic=GAZEBO_RUDDER_CMD_TOPIC,
            qos_profile=qos_depth,
        )
        self.__gazebo_sail_cmd_pub = self.create_publisher(
            msg_type=Float64,
            topic=GAZEBO_SAIL_TRIM_TAB_CMD_TOPIC,
            qos_profile=qos_depth,
        )

    def __init_action_clients(self):
        """Initializes the action clients of this node. The action servers live in
        boat_simulator's low_level_control_node, which is shared between physics backends.
        """
        self.get_logger().debug("Initializing action clients...")
        self.__rudder_actuation_action_client = ActionClient(
            node=self,
            action_type=SimRudderActuation,
            action_name=Constants.ACTION_NAMES.RUDDER_ACTUATION,
        )
        self.__sail_actuation_action_client = ActionClient(
            node=self,
            action_type=SimSailTrimTabActuation,
            action_name=Constants.ACTION_NAMES.SAIL_ACTUATION,
        )

    def __init_timer_callbacks(self):
        """Initializes timer callbacks of this node."""
        self.get_logger().debug("Initializing timer callbacks...")
        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value
        self.create_timer(timer_period_sec=pub_period_sec, callback=self.__publish)
        self.create_timer(
            timer_period_sec=self.get_parameter("rudder.actuation_request_period_sec")
            .get_parameter_value()
            .double_value,
            callback=self.__rudder_action_send_goal,
        )
        self.create_timer(
            timer_period_sec=self.get_parameter("wingsail.actuation_request_period_sec")
            .get_parameter_value()
            .double_value,
            callback=self.__sail_action_send_goal,
        )

    # PUBLISHER CALLBACKS
    def __publish(self):
        """Publishes the boat state derived from the latest Gazebo odometry to the same
        topics as the kinematic physics engine.
        """
        odometry = self.__latest_odometry
        if odometry is None:
            self.get_logger().warn(
                f"No odometry received from Gazebo on {GAZEBO_ODOMETRY_TOPIC} yet; is the boat"
                + " model spawned with an OdometryPublisher plugin?",
                throttle_duration_sec=self.__info_log_throttle_period_sec(),
            )
            return

        gps_msg = self.__publish_gps(odometry)
        self.__publish_wind_sensors(odometry)
        self.__publish_kinematics(odometry, gps_msg)
        self.__publish_counter += 1

    def __publish_gps(self, odometry: Odometry) -> GPS:
        """Publishes GPS data derived from Gazebo odometry.

        Args:
            odometry (Odometry): The latest bridged Gazebo odometry (ENU world frame pose,
                body frame twist).

        Returns:
            GPS: The published message, reused by `__publish_kinematics` as `global_gps`.
        """
        position = odometry.pose.pose.position
        latitude, longitude = local_position_to_gps_lat_lon(
            [position.x, position.y],
            self.__gps_origin_lat,
            self.__gps_origin_lon,
        )
        speed_mps = math.hypot(odometry.twist.twist.linear.x, odometry.twist.twist.linear.y)
        heading_ccw_east_deg = math.degrees(self.__yaw_rad(odometry))

        msg = GPS()
        msg.lat_lon.latitude = float(latitude)
        msg.lat_lon.longitude = float(longitude)
        msg.speed.speed = float(ConversionFactors.mPs_to_kmPh.value.forward_convert(speed_mps))
        msg.heading.heading = float(Utils.ccw_straight_to_cw_north_deg(heading_ccw_east_deg))
        self.gps_pub.publish(msg)

        rudder_msg = HelperHeading(heading=heading_ccw_east_deg)
        self.rudder_pub.publish(rudder_msg)

        self.get_logger().debug(
            f"Publishing to {self.gps_pub.topic}",
            throttle_duration_sec=self.__info_log_throttle_period_sec(),
        )
        return msg

    def __publish_wind_sensors(self, odometry: Odometry):
        """Publishes the apparent wind computed from the configured constant true wind and
        the boat velocity from Gazebo odometry.

        Unlike the kinematic backend, no sensor noise or moving-average filtering is applied;
        Gazebo is the source of truth for boat motion and the world wind is constant.

        Args:
            odometry (Odometry): The latest bridged Gazebo odometry.
        """
        yaw_rad = self.__yaw_rad(odometry)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        # Body (x forward, y port) twist rotated into the ENU world frame.
        surge_mps = odometry.twist.twist.linear.x
        sway_mps = odometry.twist.twist.linear.y
        boat_velocity_east = surge_mps * cos_yaw - sway_mps * sin_yaw
        boat_velocity_north = surge_mps * sin_yaw + sway_mps * cos_yaw

        # Apparent (flow-toward) wind in ENU, then projected onto the boat's forward and
        # starboard axes.
        apparent_east = self.__true_wind_enu_mps[0] - boat_velocity_east
        apparent_north = self.__true_wind_enu_mps[1] - boat_velocity_north
        apparent_forward = apparent_east * cos_yaw + apparent_north * sin_yaw
        apparent_starboard = apparent_east * sin_yaw - apparent_north * cos_yaw

        # WindSensor.direction: flow-toward, 0 degrees = bow-to-stern flow, increases CW
        # (stern -> port -> bow -> starboard).
        direction_deg = math.degrees(math.atan2(-apparent_starboard, -apparent_forward))
        speed_mps = math.hypot(apparent_forward, apparent_starboard)

        msg = WindSensor()
        msg.speed.speed = float(ConversionFactors.mPs_to_kmPh.value.forward_convert(speed_mps))
        msg.direction = int(Utils.bound_to_180(round(direction_deg)))
        self.wind_sensors_pub.publish(msg)
        self.get_logger().debug(
            f"Publishing to {self.wind_sensors_pub.topic}",
            throttle_duration_sec=self.__info_log_throttle_period_sec(),
        )

    def __publish_kinematics(self, odometry: Odometry, gps_msg: GPS):
        """Publishes the kinematics data of the simulated boat.

        Args:
            odometry (Odometry): The latest bridged Gazebo odometry.
            gps_msg (GPS): The message published by `__publish_gps` this tick, embedded as
                `SimWorldState.global_gps`.
        """
        mps_to_kmph = ConversionFactors.mPs_to_kmPh.value.forward_convert
        m_to_km = ConversionFactors.m_to_km.value.forward_convert

        msg = SimWorldState()
        msg.global_gps = gps_msg

        position = odometry.pose.pose.position
        msg.global_pose.position.x = float(m_to_km(position.x))
        msg.global_pose.position.y = float(m_to_km(position.y))
        msg.global_pose.position.z = float(m_to_km(position.z))
        msg.global_pose.orientation = odometry.pose.pose.orientation

        msg.wind_velocity.x = float(mps_to_kmph(self.__true_wind_enu_mps[0]))
        msg.wind_velocity.y = float(mps_to_kmph(self.__true_wind_enu_mps[1]))
        msg.wind_velocity.z = 0.0

        # The Gazebo world has no ocean current model; report zero current.
        msg.current_velocity.x = 0.0
        msg.current_velocity.y = 0.0
        msg.current_velocity.z = 0.0

        msg.header.stamp = odometry.header.stamp
        msg.header.frame_id = str(self.__publish_counter)

        self.kinematics_pub.publish(msg)
        self.get_logger().debug(
            f"Publishing to {self.kinematics_pub.topic}",
            throttle_duration_sec=self.__info_log_throttle_period_sec(),
        )

    # SUBSCRIPTION CALLBACKS
    def __gazebo_odometry_sub_callback(self, msg: Odometry):
        """Stores the latest odometry from Gazebo.

        Args:
            msg (Odometry): The bridged Gazebo odometry.
        """
        self.__latest_odometry = msg

    def __desired_heading_sub_callback(self, msg: DesiredHeading):
        """Stores the latest desired heading data.

        Args:
            msg (DesiredHeading): The desired heading data from local pathfinding.
        """
        self.get_logger().info(
            f"Received data from {self.__desired_heading_sub.topic}",
            throttle_duration_sec=self.__info_log_throttle_period_sec(),
        )
        self.__desired_heading = msg

    def __sail_trim_tab_angle_sub_callback(self, msg: SailCmd):
        """Stores the latest trim tab angle from the controller.

        Args:
            msg (SailCmd): The desired trim tab angle.
        """
        self.get_logger().info(
            f"Received data from {self.__sail_trim_tab_angle_sub.topic}",
            throttle_duration_sec=self.__info_log_throttle_period_sec(),
        )
        self.__sail_trim_tab_angle_deg = msg.trim_tab_angle_degrees

    # RUDDER ACTUATION ACTION CLIENT CALLBACKS
    def __rudder_action_send_goal(self):
        """Asynchronously sends a goal request to the rudder actuation action server."""
        if self.__desired_heading is None:
            self.get_logger().warn(
                "No desired heading received yet; skipping rudder actuation request"
            )
            return
        self.get_logger().debug("Initiating goal request for rudder actuation action")

        goal_msg = SimRudderActuation.Goal()
        goal_msg.desired_heading = self.__desired_heading

        timeout_sec = (
            self.get_parameter("action_send_goal_timeout_sec").get_parameter_value().double_value
        )
        if not self.__rudder_actuation_action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"Rudder actuation action goal request timed out after {timeout_sec} seconds."
                + " Aborting..."
            )
            return
        send_goal_future = self.__rudder_actuation_action_client.send_goal_async(
            goal=goal_msg, feedback_callback=self.__rudder_action_feedback_callback
        )
        send_goal_future.add_done_callback(self.__rudder_action_goal_response_callback)

    def __rudder_action_goal_response_callback(self, future: Future):
        """Handles the acknowledgement of a rudder actuation goal request.

        Args:
            future (Future): The outcome of the goal request in the future.
        """
        goal_handle: Optional[ClientGoalHandle] = future.result()
        if (not goal_handle) or (not goal_handle.accepted):
            self.get_logger().warn("Attempted to send rudder actuation goal, but it was rejected")

    def __rudder_action_feedback_callback(self, feedback_msg: SimRudderActuation_FeedbackMessage):
        """Forwards rudder angle feedback to the Gazebo rudder joint controller.

        The action feedback angle increases CW (viewed from above), while the model
        contract's `rudder_joint` rotates about +Z (CCW positive), so the sign is flipped.

        Args:
            feedback_msg (SimRudderActuation_FeedbackMessage): The feedback message.
        """
        self.__rudder_angle_deg = feedback_msg.feedback.rudder_angle
        cmd = Float64()
        cmd.data = -Utils.degrees_to_rad(self.__rudder_angle_deg)
        self.gazebo_rudder_cmd_pub.publish(cmd)

    # SAIL ACTUATION ACTION CLIENT CALLBACKS
    def __sail_action_send_goal(self):
        """Asynchronously sends a goal request to the sail actuation action server."""
        self.get_logger().debug("Initiating goal request for sail actuation action")

        goal_msg = SimSailTrimTabActuation.Goal()
        goal_msg.desired_angular_position = self.__sail_trim_tab_angle_deg

        timeout_sec = (
            self.get_parameter("action_send_goal_timeout_sec").get_parameter_value().double_value
        )
        if not self.__sail_actuation_action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().warn(
                f"Sail actuation action goal request timed out after {timeout_sec} seconds."
                + " Aborting..."
            )
            return
        send_goal_future = self.__sail_actuation_action_client.send_goal_async(
            goal=goal_msg, feedback_callback=self.__sail_action_feedback_callback
        )
        send_goal_future.add_done_callback(self.__sail_action_goal_response_callback)

    def __sail_action_goal_response_callback(self, future: Future):
        """Handles the acknowledgement of a sail actuation goal request.

        Args:
            future (Future): The outcome of the goal request in the future.
        """
        goal_handle: Optional[ClientGoalHandle] = future.result()
        if (not goal_handle) or (not goal_handle.accepted):
            self.get_logger().warn("Attempted to send sail actuation goal, but it was rejected")

    def __sail_action_feedback_callback(
        self, feedback_msg: SimSailTrimTabActuation_FeedbackMessage
    ):
        """Forwards trim tab angle feedback to the Gazebo trim tab joint controller.

        The feedback angle increases CCW, matching the model contract's +Z `trim_tab_joint`
        axis, so no sign flip is needed.

        Args:
            feedback_msg (SimSailTrimTabActuation_FeedbackMessage): The feedback message.
        """
        cmd = Float64()
        cmd.data = Utils.degrees_to_rad(feedback_msg.feedback.current_angular_position)
        self.gazebo_sail_cmd_pub.publish(cmd)

    # HELPERS
    @staticmethod
    def __yaw_rad(odometry: Odometry) -> float:
        """Extracts the yaw (rotation about ENU +Z, CCW from east) from an odometry message.

        Args:
            odometry (Odometry): The odometry message.

        Returns:
            float: The yaw angle in radians.
        """
        q = odometry.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def __info_log_throttle_period_sec(self) -> float:
        return (
            self.get_parameter("info_log_throttle_period_sec")
            .get_parameter_value()
            .double_value
        )

    # CLASS PROPERTY PUBLIC GETTERS
    @property
    def gps_pub(self):
        return self.__gps_pub

    @property
    def rudder_pub(self):
        return self.__rudder_pub

    @property
    def wind_sensors_pub(self):
        return self.__wind_sensors_pub

    @property
    def kinematics_pub(self):
        return self.__kinematics_pub

    @property
    def gazebo_rudder_cmd_pub(self):
        return self.__gazebo_rudder_cmd_pub

    @property
    def gazebo_sail_cmd_pub(self):
        return self.__gazebo_sail_cmd_pub

    @property
    def desired_heading(self) -> Optional[DesiredHeading]:
        return self.__desired_heading

    @property
    def rudder_angle_deg(self) -> float:
        return self.__rudder_angle_deg

    @property
    def sail_trim_tab_angle_deg(self) -> float:
        return self.__sail_trim_tab_angle_deg

    @property
    def latest_odometry(self) -> Optional[Odometry]:
        return self.__latest_odometry

    @property
    def true_wind_enu_mps(self) -> List[float]:
        return list(self.__true_wind_enu_mps)


if __name__ == "__main__":
    main()
