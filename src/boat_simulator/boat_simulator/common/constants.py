"""Constants used across the boat simulator package."""

import os
from dataclasses import dataclass
from enum import Enum

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.types import Scalar


# Class declarations for constants. These are not meant to be accessed directly.
@dataclass
class Actions:
    RUDDER_ACTUATION: str = "rudder_actuation"
    SAIL_ACTUATION: str = "sail_trim_tab_actuation"


@dataclass
class LowLevelControlSubscriptionTopics:
    GPS: str = "gps"


@dataclass
class PhysicsEngineSubscriptionTopics:
    DESIRED_HEADING: str = "desired_heading"
    SAIL_TRIM_TAB_ANGLE: str = "sail_cmd"


@dataclass
class PhysicsEnginePublisherTopics:
    GPS: str = "gps"
    KINEMATICS: str = "mock_kinematics"
    WIND_SENSORS: str = "wind_sensors"


@dataclass
class BoatProperties:
    # A lookup table of shape [N, 2] where each row is [angle_of_attack_deg, lift_coefficient].
    sail_lift_coeffs: NDArray
    # A lookup table of shape [N, 2] where each row is [angle_of_attack_deg, drag_coefficient].
    sail_drag_coeffs: NDArray
    # A lookup table of shape [N, 2] where each row is [angle_of_attack_deg, sail_area_m2].
    sail_areas: NDArray
    # A lookup table of shape [N, 2] where each row is
    # [angle_of_attack_deg, rudder_drag_coefficient].
    rudder_lift_coeffs: NDArray
    # A lookup table of shape [N, 2] where each row is
    # [angle_of_attack_deg, rudder_drag_coefficient].
    rudder_drag_coeffs: NDArray
    # A lookup table of shape [N, 2] where each row is [angle_of_attack_deg, rudder_area_m2].
    rudder_areas: NDArray
    # A scalar representing the distance from the center of effort of the sail to the pivot point
    # (in meters).
    sail_dist: Scalar
    # A scalar representing the distance from the center of effort of the rudder to the pivot
    # point (in meters).
    rudder_dist: Scalar
    # A dimensionless scalar representing the drag factor of the hull as a function of the boat's
    # velocity.
    hull_drag_factor: Scalar
    # The mass of the boat (in kilograms).
    mass: Scalar
    # The inertia of the boat (in kilograms-meters squared).
    inertia: NDArray


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

# CLI argument name for mock data option
MOCK_DATA_CLI_ARG_NAME = "--enable-mock-data"
SIM_VISUALIZER_CLI_ARG_NAME = "--enable-sim-visualizer"

# Enumerated orientation indices since indexing pitch, roll, and yaw could be arbitrary
ORIENTATION_INDICES = Enum("ORIENTATION_INDICES", ["PITCH", "ROLL", "YAW"], start=0)  # x, y, x

# Number of times the sail action server routine's main loop executes
SAIL_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the ctrl is integrated

# Number of times the rudder action server routine's main loop executes
RUDDER_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the PID is integrated

# Max rudder control angle range in degrees, min angle [0] and max angle [1]
RUDDER_MAX_ANGLE_RANGE = (-45, 45)

# Max sail actuator control angle range in degrees, min angle [0], max angle [1]
SAIL_MAX_ANGLE_RANGE = (-7, 7)

# Densities of the mediums, used for force calculations, units in kg/m^3
AIR_DENSITY = 1.225
WATER_DENSITY = 1027.0

# Constants related to the physical and mechanical properties of Polaris
# TODO These are placeholder values which should be replaced when we have real values.
BOAT_PROPERTIES = BoatProperties(
    sail_lift_coeffs=np.array(
        [[0.0, 0.0], [5.0, 0.2], [10.0, 0.5], [15.0, 0.7], [20.0, 1.0]], dtype=np.float32
    ),
    sail_drag_coeffs=np.array(
        [[0.0, 0.1], [5.0, 0.12], [10.0, 0.15], [15.0, 0.18], [20.0, 0.2]], dtype=np.float32
    ),
    sail_areas=np.array(
        [[0.0, 20.0], [5.0, 19.8], [10.0, 19.5], [15.0, 19.2], [20.0, 18.8]], dtype=np.float32
    ),
    rudder_lift_coeffs=np.array(
        [
            [-20.0, -0.9],
            [-15.0, -0.85],
            [-10.0, -0.6],
            [-5.0, -0.3],
            [0.0, 0.0],
            [5.0, 0.3],
            [10.0, 0.6],
            [15.0, 0.85],
            [20.0, 0.9],
        ],
        dtype=np.float32,
    ),
    rudder_drag_coeffs=np.array(
        [[0.0, 0.2], [5.0, 0.22], [10.0, 0.25], [15.0, 0.28], [20.0, 0.3]], dtype=np.float32
    ),
    rudder_areas=np.array(
        [[0.0, 2.0], [5.0, 1.9], [10.0, 1.8], [15.0, 1.7], [20.0, 1.6]], dtype=np.float32
    ),
    sail_dist=0.5,
    rudder_dist=1.0,
    hull_drag_factor=0.05,
    mass=1500.0,
    inertia=np.array([[125, 0, 0], [0, 1125, 0], [0, 0, 500]], dtype=np.float32),
)
