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

    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package=PACKAGE_NAME,
                    executable="mock_wind_sensor",
                    name="mock_wind_sensor",
                ),

                # Fake node
                launch_ros.actions.Node(
                    package=PACKAGE_NAME,
                    executable="fake_gps",
                    name="fake_gps",
                ),


                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
                ]
            ), {},
    )
