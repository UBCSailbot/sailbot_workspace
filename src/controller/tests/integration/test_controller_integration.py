import time
import unittest

import launch_ros.actions
import launch_testing.actions
import rclpy
from custom_interfaces.msg import SailCmd, WindSensor
from rclpy.node import Node

import launch


def generate_test_description():
    wingsail_node = launch_ros.actions.Node(
        package="controller",
        executable="wingsail_ctrl_node",
        parameters=["/workspaces/sailbot_workspace/src/global_launch/config/globals.yaml"],
    )
    return launch.LaunchDescription([wingsail_node, launch_testing.actions.ReadyToTest()])


class WindSensorPublisher(Node):
    def __init__(self):
        super().__init__("wind_sensor_publisher")
        self.publisher = self.create_publisher(WindSensor, "filtered_wind_sensor", 1)
        self.received_msg = None
        self.subscription = self.create_subscription(
            SailCmd, "sail_cmd", self.sail_cmd_callback, 1
        )

    def sail_cmd_callback(self, msg):
        self.received_msg = msg

    def publish_wind(self, speed, direction):
        msg = WindSensor()
        msg.speed.speed = float(speed)
        msg.direction = direction
        self.publisher.publish(msg)


class TestWingsailSafetyLogic(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = WindSensorPublisher()
        time.sleep(2.0)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_above_upper_threshold(self):
        self.node.publish_wind(speed=20.0, direction=90)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        time.sleep(2.0)
        rclpy.spin_once(self.node, timeout_sec=2.0)
        self.assertIsNotNone(self.node.received_msg)
        self.assertEqual(self.node.received_msg.trim_tab_angle_degrees, 0.0)

    def test_between_thresholds(self):
        self.node.publish_wind(speed=12.0, direction=90)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        time.sleep(2.0)
        rclpy.spin_once(self.node, timeout_sec=2.0)
        self.assertIsNotNone(self.node.received_msg)
        self.assertGreater(self.node.received_msg.trim_tab_angle_degrees, 0.0)

    def test_below_lower_threshold(self):
        self.node.publish_wind(speed=5.0, direction=90)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        time.sleep(2.0)
        rclpy.spin_once(self.node, timeout_sec=2.0)
        self.assertIsNotNone(self.node.received_msg)
        self.assertGreater(self.node.received_msg.trim_tab_angle_degrees, 0.0)
