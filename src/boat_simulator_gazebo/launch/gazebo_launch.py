"""Launch file for the Gazebo physics backend of the boat simulator.

Starts the Ignition Gazebo server with the sailing world, the ros_gz parameter bridge,
optionally spawns the boat model, and runs gazebo_physics_bridge_node, which adapts
Gazebo's topics to the physics engine's ROS contract.

Lifetime is tied together in both directions: if the Gazebo server or the bridge node
exits, the whole launch shuts down, so a dead simulator never leaves the ROS side running
against a frozen world and vice versa.
"""

import importlib.util
import os
from typing import List

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "boat_simulator_gazebo"
ROS_WORKSPACE = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace")
PACKAGE_SRC_DIR = os.path.join(ROS_WORKSPACE, "src", PACKAGE_NAME)

# Model name expected by the world's Buoyancy system and the ros_gz_bridge topic map.
BOAT_MODEL_NAME = "polaris"

# Topics bridged between Gazebo and ROS, as (parameter_bridge spec, ROS topic name).
#
# The ros_gz_bridge shipped for Humble predates YAML config files, so each bridge
# is declared as a CLI spec of the form `<topic>@<ROS type><direction><Gazebo type>`, where
# the direction character is `[` for Gazebo->ROS and `]` for ROS->Gazebo. The spec's topic
# is the Gazebo name, which is also what the bridge would call the ROS topic, so the second
# tuple element is the ROS name we remap it to.
#
# The Gazebo names are part of the boat model contract (see this package's README): the
# model must be named `polaris` and carry an OdometryPublisher plugin plus a
# JointPositionController per actuated joint. The ROS names are what
# gazebo_physics_bridge_node subscribes to and publishes on.
BRIDGED_TOPICS = [
    (
        "/model/polaris/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/gazebo/odometry",
    ),
    (
        "/model/polaris/joint/rudder_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
        "/gazebo/rudder_angle_cmd",
    ),
    (
        "/model/polaris/joint/trim_tab_joint/cmd_pos@std_msgs/msg/Float64"
        + "]gz.msgs.Double",
        "/gazebo/sail_trim_tab_angle_cmd",
    ),
    ("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock", "/gazebo/clock"),
]


def gz_name(bridge_spec: str) -> str:
    """Extracts the topic name from a parameter_bridge CLI spec.

    Args:
        bridge_spec (str): A spec of the form `<topic>@<ROS type><direction><Gazebo type>`.

    Returns:
        str: The topic name preceding the first `@`.
    """
    return bridge_spec.split("@", 1)[0]


# Height of the boat's centre of gravity above the water surface at spawn.
#
# This is a property of the model, not of the launch, because it is the model's flotation
# equilibrium: spawn a boat here and it starts at rest instead of dropping in and bobbing.
# Both shipped models put their origin on the CG, but they reach equilibrium at different
# CG heights, so the default below only matches the default model:
#
#   models/polaris_basic  0.2   hull collision box 3.6 x 0.99 x 0.55 m centred on the CG,
#                               displacing mass / WATER_DENSITY = 276 / 1027 = 0.269 m^3
#                               at a 0.075 m draft, so the CG sits 0.55/2 - 0.075 above
#                               the waterline.
#   models/polaris        0.0   uses the same collision box, raised on the CG so the
#                               waterline lands at z = -0.006, which is where the real
#                               hull mesh floats at this displacement.
#
# Override with the boat_spawn_height launch argument when spawning a different model.
DEFAULT_BOAT_SPAWN_HEIGHT_M = 0.2

LOCAL_LAUNCH_ARGUMENTS: List[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        name="gz_world",
        default_value=os.path.join(PACKAGE_SRC_DIR, "worlds", "sailing_world.sdf"),
        description="Path to the Gazebo world SDF file",
    ),
    DeclareLaunchArgument(
        name="gz_headless",
        default_value="false",
        choices=["true", "false"],
        description="Run the Gazebo server without the GUI (required in the Dev Container"
        + " unless a display is available)",
    ),
    DeclareLaunchArgument(
        name="boat_model_sdf",
        default_value=os.path.join(PACKAGE_SRC_DIR, "models", "polaris_basic", "model.sdf"),
        description="Path to the boat model SDF/URDF satisfying the model contract in"
        + f" src/{PACKAGE_NAME}/README.md. Defaults to models/polaris_basic, the"
        + " contract-verified box stand-in; models/polaris is the same boat drawn with"
        + " the real CAD hull mesh (pass boat_spawn_height:=0.0 with it)."
        + " Set to an empty string to launch the world without a boat",
    ),
    DeclareLaunchArgument(
        name="boat_spawn_height",
        default_value=str(DEFAULT_BOAT_SPAWN_HEIGHT_M),
        description="Height of the boat model's origin above the mean water level at spawn,"
        + " in metres. The default is models/polaris_basic's flotation equilibrium; use 0.0"
        + " for models/polaris",
    ),
]


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the
    `boat_simulator_gazebo` package.

    Returns:
        LaunchDescription: The launch description.
    """
    global_launch_arguments, global_environment_vars = get_global_launch_arguments()
    return LaunchDescription(
        [
            *global_launch_arguments,
            *global_environment_vars,
            *LOCAL_LAUNCH_ARGUMENTS,
            OpaqueFunction(function=setup_launch),
        ]
    )


def get_global_launch_arguments() -> tuple:
    """Gets the global launch arguments and environment variables from the global launch file.

    Returns:
        Tuple: The global launch arguments and environment variables.
    """
    global_main_launch = os.path.join(ROS_WORKSPACE, "src", "global_launch", "main_launch.py")
    module = import_module_from_source("global_launch", global_main_launch)
    return module.GLOBAL_LAUNCH_ARGUMENTS, module.ENVIRONMENT_VARIABLES


def import_module_from_source(module_name: str, source_path: str):
    """Imports a Python module directly from a source file path.

    Args:
        module_name (str): Name to register the module under.
        source_path (str): Absolute path to the module's source file.

    Returns:
        ModuleType: The imported module.
    """
    spec = importlib.util.spec_from_file_location(module_name, source_path)
    if spec is None:
        raise ImportError(f"Couldn't import {module_name} module from {source_path}")
    module = importlib.util.module_from_spec(spec)  # type: ignore[arg-type] # spec is not None
    spec.loader.exec_module(module)  # type: ignore[union-attr] # spec is not None
    return module


def setup_launch(context: LaunchContext) -> List:
    """Collects launch descriptions that describe the system behavior in the
    `boat_simulator_gazebo` package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List: Launch actions to execute.
    """
    config = LaunchConfiguration("config").perform(context)
    if not os.path.isabs(config):
        config = os.path.join(ROS_WORKSPACE, "src", "global_launch", "config", config)
        context.launch_configurations["config"] = config

    physics_bridge = get_gazebo_physics_bridge_description(context)
    launch_description_entities = [
        get_gazebo_server_description(context),
        get_ros_gz_bridge_description(context),
        physics_bridge,
        # The physics bridge is what makes this backend a physics engine as far as the rest
        # of the system is concerned. If it dies, shut down so the Gazebo server goes with
        # it rather than lingering and holding its transport topics.
        RegisterEventHandler(
            OnProcessExit(
                target_action=physics_bridge,
                on_exit=[Shutdown(reason="gazebo_physics_bridge_node exited")],
            )
        ),
    ]
    boat_model_sdf = LaunchConfiguration("boat_model_sdf").perform(context)
    if boat_model_sdf:
        launch_description_entities.append(get_spawn_boat_description(context, boat_model_sdf))
    return launch_description_entities


def asv_wave_sim_roots() -> List[str]:
    """Finds the asv_wave_sim trees supplying the wave field and Hydrodynamics plugin.

    The Dev Container image prebuilds asv_wave_sim into `ASV_WAVE_SIM_PREFIX`
    (/opt/asv_wave_sim), so that is preferred. A source checkout under `src/` is used as a
    fallback, which covers both an older container and anyone building it in-workspace to
    hack on the wave physics.

    Returns:
        List[str]: Existing prefixes, most preferred first. Empty if none is installed.
    """
    candidates = [
        os.getenv("ASV_WAVE_SIM_PREFIX", "/opt/asv_wave_sim"),
        os.path.join(ROS_WORKSPACE, "src", "asv_wave_sim"),
    ]
    return [path for path in candidates if os.path.isdir(path)]


def gazebo_resource_env() -> dict:
    """Builds the environment overrides the Gazebo server needs to resolve its resources.

    The world includes `model://waves` from asv_wave_sim, whose models live outside any
    ament share directory, so `GZ_SIM_RESOURCE_PATH` has to point at them explicitly. The
    prebuilt image already exports this, but it is rebuilt here so an in-workspace checkout
    works too. Existing entries are preserved, so a caller can add their own model paths.

    This package's `models/` directory is included as well, so the shipped boat models
    resolve as `model://polaris` / `model://polaris_basic` for anyone including them from a
    custom world, rather than only via the absolute path `boat_model_sdf` passes to
    `ros_gz_sim create -file`.

    Returns:
        dict: Environment variables to overlay onto the Gazebo server process.
    """
    resource_paths = [PACKAGE_SRC_DIR, os.path.join(PACKAGE_SRC_DIR, "models")]
    for root in asv_wave_sim_roots():
        models = os.path.join(root, "gz-waves-models")
        resource_paths.append(os.path.join(models, "world_models"))
        resource_paths.append(os.path.join(models, "models"))

    existing = os.getenv("GZ_SIM_RESOURCE_PATH", "")
    if existing:
        resource_paths.append(existing)
    return {"GZ_SIM_RESOURCE_PATH": os.pathsep.join(resource_paths)}


def get_gazebo_server_description(context: LaunchContext) -> ExecuteProcess:
    """Gets the process description that starts the Gazebo (gz-sim) server.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        ExecuteProcess: The process object that runs `gz sim`.
    """
    world_path = LaunchConfiguration("gz_world").perform(context)
    headless = LaunchConfiguration("gz_headless").perform(context) == "true"
    # -r: start the simulation running; -s: server only (no GUI).
    cmd = ["gz", "sim", "-r"]
    if headless:
        cmd.append("-s")
    cmd.append(world_path)
    return ExecuteProcess(
        cmd=cmd,
        output="screen",
        shell=False,
        additional_env=gazebo_resource_env(),
        # The server runs inside the `gz` process itself (the CLI is a Ruby wrapper that
        # loads the server via FFI), so signalling this process is what stops the
        # simulation. It does not always exit on SIGINT while running headless, hence the
        # explicit escalation to SIGTERM and then SIGKILL; without it the server outlives
        # the launch and the next run fails to bind its transport topics.
        sigterm_timeout="5",
        sigkill_timeout="5",
        # A dead simulator makes the remaining nodes meaningless, so tear everything down.
        on_exit=Shutdown(reason="Gazebo server exited"),
    )


def get_ros_gz_bridge_description(context: LaunchContext) -> Node:
    """Gets the launch description for the ros_gz parameter bridge.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that bridges Gazebo and ROS topics.
    """
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_parameter_bridge",
        arguments=[spec for spec, _ros_name in BRIDGED_TOPICS],
        # parameter_bridge names each ROS topic after its Gazebo counterpart, so the
        # ROS-side names in the contract are produced by remapping.
        remappings=[(gz_name(spec), ros_name) for spec, ros_name in BRIDGED_TOPICS],
        ros_arguments=[
            "--log-level",
            ["ros_gz_parameter_bridge:=", LaunchConfiguration("log_level")],
        ],
    )


def get_spawn_boat_description(context: LaunchContext, boat_model_sdf: str) -> Node:
    """Gets the launch description that spawns the boat model into the running world.

    Args:
        context (LaunchContext): The current launch context.
        boat_model_sdf (str): Path to the boat model SDF/URDF file.

    Returns:
        Node: The node object that runs `ros_gz_sim create`.
    """
    return Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_boat",
        arguments=[
            "-file",
            boat_model_sdf,
            "-name",
            BOAT_MODEL_NAME,
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            LaunchConfiguration("boat_spawn_height"),
        ],
    )


def get_gazebo_physics_bridge_description(context: LaunchContext) -> Node:
    """Gets the launch description for the Gazebo physics bridge node.

    The node reuses the physics engine's GPS origin parameters, which are resolved from the
    selected PATH test plan by the helpers in boat_simulator's launch file.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches gazebo_physics_bridge_node.
    """
    node_name = "gazebo_physics_bridge_node"
    config_path = LaunchConfiguration("config").perform(context)

    # Reuse boat_simulator's launch helpers so the GPS origin comes from the same test plan
    # resolution logic as the kinematic backend.
    boat_simulator_launch = import_module_from_source(
        "boat_simulator_main_launch",
        os.path.join(ROS_WORKSPACE, "src", "boat_simulator", "launch", "main_launch.py"),
    )
    test_plan = LaunchConfiguration("test_plan").perform(context)
    if not test_plan:
        test_plan = boat_simulator_launch.get_test_plan_from_config(config_path)
    ros_parameters = [
        config_path,
        boat_simulator_launch.get_sim_gps_origin_parameters(test_plan),
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    return Node(
        package=PACKAGE_NAME,
        executable=node_name,
        name=node_name,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )
