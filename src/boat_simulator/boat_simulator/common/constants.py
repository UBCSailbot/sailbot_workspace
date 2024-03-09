"""Constants used across the boat simulator package."""

import os
from dataclasses import dataclass
from enum import Enum


# Class declarations for constants. These are not meant to be accessed directly.
@dataclass
class Actions:
    RUDDER_ACTUATION: str = "rudder_actuation"
    SAIL_ACTUATION: str = "sail_trim_tab_actuation"


@dataclass
class LowLevelControlSubscriptionTopics:
    GPS: str = "mock_gps"


@dataclass
class PhysicsEngineSubscriptionTopics:
    DESIRED_HEADING: str = "desired_heading"
    SAIL_TRIM_TAB_ANGLE: str = "sail_cmd"


@dataclass
class PhysicsEnginePublisherTopics:
    GPS: str = "mock_gps"
    KINEMATICS: str = "mock_kinematics"
    WIND_SENSORS: str = "mock_wind_sensors"


# Directly accessible constants

# Boat simulator ROS action names
ACTION_NAMES = Actions()

# Base directory to store the output data from the data collection node
DATA_COLLECTION_OUTPUT_DIR = os.path.join(str(os.getenv("ROS_WORKSPACE")), "boat_simulator_output")

# ROS topic names for the low level control node subscriptions
LOW_LEVEL_CTRL_SUBSCRIPTIONS = LowLevelControlSubscriptionTopics()

# CLI argument name for multithreading option for physics engine
MULTITHREADING_CLI_ARG_NAME = "--enable-multithreading"

# ROS topic names for physics engine publishers
PHYSICS_ENGINE_PUBLISHERS = PhysicsEnginePublisherTopics()

# ROS topic names for physics engine subscriptions
PHYSICS_ENGINE_SUBSCRIPTIONS = PhysicsEngineSubscriptionTopics()

# CLI argument name for data collection option
DATA_COLLECTION_CLI_ARG_NAME = "--enable-data-collection"

# Enumerated orientation indices since indexing pitch, roll, and yaw could be arbitrary
ORIENTATION_INDICES = Enum("ORIENTATION_INDICES", ["PITCH", "ROLL", "YAW"], start=0)  # x, y, x

# Number of times the sail action server routine's main loop executes
SAIL_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the ctrl is integrated

# Number of times the rudder action server routine's main loop executes
RUDDER_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the PID is integrated
