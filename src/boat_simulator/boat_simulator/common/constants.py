"""Constants used across the boat simulator package."""

import os
from dataclasses import dataclass
from enum import Enum

import numpy as np

from boat_simulator.common.conventions import Body, Inertia
from boat_simulator.common.types import CoeffTable, Mat3


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
    KINEMATICS: str = "kinematics"
    FILTERED_WIND_SENSORS: str = "filtered_wind_sensor"


@dataclass
class BoatProperties:
    # Shape [N, 2]: each row is [angle_of_attack_deg, lift_coefficient].
    sail_lift_coeffs: CoeffTable
    # Shape [N, 2]: each row is [angle_of_attack_deg, drag_coefficient].
    sail_drag_coeffs: CoeffTable
    # Float: each row is sail_area_m2.
    sail_areas: float
    # Shape [N, 2]: each row is [angle_of_attack_deg, lift_coefficient].
    rudder_lift_coeffs: CoeffTable
    # Shape [N, 2]: each row is [angle_of_attack_deg, drag_coefficient].
    rudder_drag_coeffs: CoeffTable
    # Float: each row is rudder_area_m2.
    rudder_areas: float
    # Distance from the center of effort of the sail to the pivot point (m).
    sail_dist: float
    # Distance from the center of effort of the rudder to the pivot point (m).
    rudder_dist: float
    # Dimensionless quadratic drag coefficient for the hull: F_drag = hull_drag_factor * |v| * v.
    hull_drag_factor: float
    # Mass of the boat (kg).
    mass: float
    # 3×3 body-frame inertia tensor (kg·m²): rows/cols are roll, pitch, yaw axes.
    inertia: Mat3[Inertia, Body]


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

# ROS parameter names for the simulator GPS origin.
SIM_GPS_ORIGIN_LATITUDE_PARAM = "sim_gps_origin_latitude"
SIM_GPS_ORIGIN_LONGITUDE_PARAM = "sim_gps_origin_longitude"

# CLI argument name for data collection option
DATA_COLLECTION_CLI_ARG_NAME = "--enable-data-collection"

# CLI argument name for mock data option
MOCK_DATA_CLI_ARG_NAME = "--enable-mock-data"
SIM_VISUALIZER_CLI_ARG_NAME = "--enable-sim-visualizer"

# Enumerated orientation indices since indexing pitch, roll, and yaw could be arbitrary
ORIENTATION_INDICES = Enum("ORIENTATION_INDICES", ["PITCH", "ROLL", "YAW"], start=0)  # x, y, z

# Number of times the sail action server routine's main loop executes
SAIL_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the ctrl is integrated

# Number of times the rudder action server routine's main loop executes
RUDDER_ACTUATION_NUM_LOOP_EXECUTIONS = 10  # TODO This is a placeholder until the PID is integrated

# Max rudder control angle range in degrees, min angle [0] and max angle [1]
RUDDER_MAX_ANGLE_RANGE = (-30, 30)

# Max sail actuator control angle range in degrees, min angle [0], max angle [1]
SAIL_MAX_ANGLE_RANGE = (-40, 40)

# Densities of the mediums, used for force calculations, units in kg/m^3
AIR_DENSITY = 1.225
WATER_DENSITY = 1027.0

# Constants related to the physical and mechanical properties of Polaris
# TODO These are placeholder values which should be replaced when we have real values.
BOAT_PROPERTIES = BoatProperties(
    # Sail: angle of attack 0–90° (wingsail, CL peaks ~25° then stalls)
    sail_lift_coeffs=CoeffTable(
        np.array(
            [
                [0.0, 0.00],
                [5.0, 0.20],
                [10.0, 0.55],
                [15.0, 0.85],
                [20.0, 1.05],
                [25.0, 1.20],  # peak lift
                [30.0, 1.10],  # stall onset
                [40.0, 0.80],
                [50.0, 0.60],
                [60.0, 0.50],
                [75.0, 0.25],
                [90.0, 0.00],  # dead downwind, pure drag
            ],
            dtype=np.float64,
        )
    ),
    sail_drag_coeffs=CoeffTable(
        np.array(
            [
                [0.0, 0.01],
                [5.0, 0.015],
                [10.0, 0.025],
                [15.0, 0.032],
                [20.0, 0.050],
                [25.0, 0.105],
                [30.0, 0.830],  # stall — drag spikes
                [40.0, 0.380],
                [50.0, 0.580],
                [60.0, 0.980],
                [75.0, 1.020],
                [90.0, 1.200],
            ],
            dtype=np.float64,
        )
    ),
    sail_areas=20.0,  # meters ^ 2
    # Rudder: ±45° → table covers 0–45° (sign handled by caller)
    # NACA symmetric foil: stalls ~20–22°
    rudder_lift_coeffs=CoeffTable(
        np.array(
            [
                [0.0, 0.00],
                [5.0, 0.30],
                [10.0, 0.60],
                [15.0, 0.85],
                [20.0, 0.92],  # peak (near stall)
                [25.0, 0.78],  # post-stall drop
                [30.0, 0.62],
                [35.0, 0.52],
                [40.0, 0.44],
                [45.0, 0.38],
            ],
            dtype=np.float64,
        )
    ),
    rudder_drag_coeffs=CoeffTable(
        np.array(
            [
                [0.0, 0.020],
                [5.0, 0.022],
                [10.0, 0.026],
                [15.0, 0.032],
                [20.0, 0.050],
                [25.0, 0.120],  # stall — drag spikes
                [30.0, 0.220],
                [35.0, 0.330],
                [40.0, 0.440],
                [45.0, 0.550],
            ],
            dtype=np.float64,
        )
    ),
    rudder_areas=0.4,  # meters ^ 2
    sail_dist=0.5,  # meters
    rudder_dist=1.0,  # meters
    hull_drag_factor=0.5,
    mass=225.0,
    inertia=Mat3(np.array([[125, 0, 0], [0, 1125, 0], [0, 0, 500]], dtype=np.float32)),
)
