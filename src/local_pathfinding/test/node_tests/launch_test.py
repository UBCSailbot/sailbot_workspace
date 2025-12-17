import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy

PACKAGE_NAME = "local_pathfinding"


def generate_test_description():

    node_name = "mock_wind_sensor"
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package=PACKAGE_NAME,
                    executable="mock_wind_sensor",
                    name=node_name,
                ),

                # Fake node
                launch_ros.actions.Node(
                    package=PACKAGE_NAME,
                    executable="",
                    name=node_name,
                ),


                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
                ]
            ), {},
    )
