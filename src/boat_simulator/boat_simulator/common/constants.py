"""Constants used across the boat simulator package."""

import os
from dataclasses import dataclass
from enum import Enum

import numpy as np

from boat_simulator.common import airfoil_polars
from boat_simulator.common.conventions import Body, Damping, Inertia
from boat_simulator.common.types import CoeffGrid, Mat4


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
    # Lift coefficient as a function of (angle_of_attack_deg, Reynolds_number).
    sail_lift_coeffs: CoeffGrid
    # Drag coefficient as a function of (angle_of_attack_deg, Reynolds_number).
    sail_drag_coeffs: CoeffGrid
    # Float: each row is sail_area_m2.
    sail_areas: float
    # Lift coefficient as a function of (angle_of_attack_deg, Reynolds_number).
    tab_lift_coeffs: CoeffGrid
    # Drag coefficient as a function of (angle_of_attack_deg, Reynolds_number).
    tab_drag_coeffs: CoeffGrid
    # Float: each row is tab_area_m2.
    tab_areas: float
    # Lift coefficient as a function of (angle_of_attack_deg, Reynolds_number).
    rudder_lift_coeffs: CoeffGrid
    # Drag coefficient as a function of (angle_of_attack_deg, Reynolds_number).
    rudder_drag_coeffs: CoeffGrid
    # Float: each row is rudder_area_m2.
    rudder_areas: float
    # Lift coefficient as a function of (angle_of_attack_deg, Reynolds_number).
    keel_lift_coeffs: CoeffGrid
    # Drag coefficient as a function of (angle_of_attack_deg, Reynolds_number).
    keel_drag_coeffs: CoeffGrid
    # Float: each row is keel_area_m2.
    keel_areas: float
    # Dimensionless quadratic drag coefficient for the hull: F_drag = hull_drag_factor * |v| * v.
    hull_drag_factor: float
    # Mass of the boat (kg).
    mass: float
    # 4×4 rigid-body generalized mass-inertia matrix M_RB = diag(m, m, I_xx, I_zz)
    # for the 4-DOF state ν = [u, v, p, r] (surge, sway, roll, yaw). Units: kg / kg·m².
    inertia: Mat4[Inertia, Body]
    # 4×4 diagonal added-mass matrix M_A = diag(X_u̇, Y_v̇, K_ṗ, N_ṙ). Units: kg / kg·m².
    M_A: Mat4[Inertia, Body]
    # 4×4 diagonal linear damping matrix D = diag(X_u, Y_v, K_p, N_r).
    # Surge/sway entries: N·s/m; roll/yaw entries: N·m·s/rad.
    D: Mat4[Damping, Body]


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

# Kinematic viscosities of the mediums, used to compute the Reynolds number
# (Re = flow_speed * chord / kinematic_viscosity) that indexes the lift/drag coefficient grids.
# Units: m^2/s. Values are for ~15 degC, consistent with AIR_DENSITY and WATER_DENSITY (seawater).
AIR_KINEMATIC_VISCOSITY = 1.48e-5
WATER_KINEMATIC_VISCOSITY = 1.05e-6

# Gravity in m/s
EARTH_GRAVITY = 9.81

# TODO: Center of Bouyancy guess. Please provide the correct point at Equilibrium
CoB_REL_COORD = 0.5

# Metacentric height used in testing
METACENTRIC_HEIGHT = 0.5  # Units: meters

# Derive the mean chord from the real wingsail geometry.
WING_SAIL_CHORD = 1.5  # Units: meters

# Mean chord lengths of the remaining foils, used only to compute each foil's Reynolds number.
# TODO: Replace these placeholders with the real measured mean chords.
TAB_CHORD = 0.3  # Units: meters
RUDDER_CHORD = 0.2  # Units: meters
KEEL_CHORD = 0.4  # Units: meters

# Measure the distance from the mast axis to the tab's aero center.
WINGSAIL_TO_TRIM_TAB_BOOM_LENGTH = 1.43  # Units: meters

# The mast pivot's chordwise position (~25% chord assumed).
MAST_PIVOT_CHORD_FRACTION = 0.25  # Fraction of the wing chord, dimensionless

# Measure the rudder's center of effort depth below the CG.
RUDDER_CE_REL_TO_CG = (-1.35, 0.74)

# The wingsail CE-to-CG in (x, Z) coordinate with units: meters
SAIL_CE_REL_TO_CG = (0.25, -1.86)  # (x_s, z_s), units: meters

# The keel's center of effort relative to the CG. The z_k is a magic number to
# maintain stability
KEEL_CE_REL_TO_CG = (0.08, -1.13)  # (x_k, z_k), units: meters

# The hull's center of Effort/Gravity relative to the boat's CG.
HULL_CE_REL_TO_CG = (0.06, 0.0, 0.40)  # (x_h, y_h, z_h), units: meters

# TODO Placeholder: measure the hull's linear drag coefficient.
HULL_LINEAR_DRAG = 0.0  # Units: newton seconds per meter


# Constants related to the physical and mechanical properties of Polaris
# TODO These are placeholder values which should be replaced when we have real values.
BOAT_PROPERTIES = BoatProperties(
    # Lift/drag coefficient grids are real XFOIL polars (airfoiltools.com) at Reynolds numbers
    # 50k–1M, resampled and Viterna-extended to 90°, generated by scripts/build_airfoil_polars.py.
    # Only the positive angle-of-attack branch is stored; MediumForceComputation applies the
    # symmetric-foil sign convention (Cl odd, Cd even) and computes the Reynolds number at runtime.
    # Wingsail and trim tab are NACA 0018; rudder and keel are NACA 0012.
    sail_lift_coeffs=airfoil_polars.SAIL_LIFT_COEFFS,
    sail_drag_coeffs=airfoil_polars.SAIL_DRAG_COEFFS,
    sail_areas=2.01,  # meters ^ 2
    tab_lift_coeffs=airfoil_polars.TAB_LIFT_COEFFS,
    tab_drag_coeffs=airfoil_polars.TAB_DRAG_COEFFS,
    tab_areas=0.198,  # meters ^ 2
    rudder_lift_coeffs=airfoil_polars.RUDDER_LIFT_COEFFS,
    rudder_drag_coeffs=airfoil_polars.RUDDER_DRAG_COEFFS,
    rudder_areas=0.117,  # meters ^ 2
    keel_lift_coeffs=airfoil_polars.KEEL_LIFT_COEFFS,
    keel_drag_coeffs=airfoil_polars.KEEL_DRAG_COEFFS,
    keel_areas=0.51,  # meters ^ 2
    hull_drag_factor=0.5,
    mass=276.0,
    # M_RB = diag(m, m, I_xx, I_zz) — surge, sway, roll, yaw
    inertia=Mat4(np.diag([225.0, 225.0, 175.86, 119.04])),
    # TODO: Replace with coefficients from strip theory or CFD for the real hull.
    # M_A = diag(X_u̇, Y_v̇, K_ṗ, N_ṙ) — diagonal linearized added-mass approximation,
    # estimated from slender-hull rules of thumb relative to M_RB:
    #   X_u̇ ≈ 0.05–0.10·m   (little water entrained fore-aft on a slender hull)
    #   Y_v̇ ≈ 0.80–1.00·m   (sway drags a large volume of water; keel adds to it)
    #   K_ṗ ≈ 0.20–0.30·I_xx
    #   N_ṙ ≈ 0.50–0.70·I_zz
    M_A=Mat4(np.diag([20.0, 400.0, 230.0, 300.0])),
    # TODO: Replace with real damping coefficients from tow-tank or CFD data.
    # D = diag(X_u, Y_v, K_p, N_r) — linear damping per DOF.
    D=Mat4(np.diag([45.0, 180.0, 115.0, 140.0])),
)


# Displaced volume of the boat at floating equilibrium (m^3)
DISPLACED_VOLUME = BOAT_PROPERTIES.mass / WATER_DENSITY
