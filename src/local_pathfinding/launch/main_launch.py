"""Launch file that runs all nodes for the local pathfinding ROS package."""

import importlib
import os
from typing import List, Tuple

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Local launch arguments and constants
PACKAGE_NAME = "local_pathfinding"

# Add args with DeclareLaunchArguments object(s) and utilize in setup_launch()
LOCAL_LAUNCH_ARGUMENTS: List[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        name="use_gps_noise",
        default_value="true",
        choices=["true", "false"],
        description="Enable Gaussian noise on GPS readings.",
    ),
    DeclareLaunchArgument(
        name="use_ocean_drift",
        default_value="true",
        choices=["true", "false"],
        description="Enable cumulative ocean current drift on GPS readings.",
    ),
    DeclareLaunchArgument(
        name="use_drift_randomization",
        default_value="true",
        choices=["true", "false"],
        description="Enable small random variation to the ocean drift current each tick.",
    ),
    DeclareLaunchArgument(
        name="ocean_drift_speed_kmph",
        default_value="0.5",
        description="Base speed of the ocean current in km/h.",
    ),
    DeclareLaunchArgument(
        name="ocean_drift_dir_deg",
        default_value="45.0",
        description="Direction the current flows toward in degrees (0=north, 90=east). Range is (-180, 180])",
    ),
    DeclareLaunchArgument(
        name="ocean_drift_accel_kmph2",
        default_value="0.0",
        description="Acceleration of the drift speed in km/h^2. Set to 0 for constant drift.",
    ),
]


def generate_launch_description() -> LaunchDescription:
    """The launch file entry point. Generates the launch description for the `local_pathfinding`
    package.

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


def get_global_launch_arguments() -> Tuple:
    """Gets the global launch arguments and environment variables from the global launch file.

    Returns:
        Tuple: The global launch arguments and environment variables.
    """
    ros_workspace = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace")
    global_main_launch = os.path.join(ros_workspace, "src", "global_launch", "main_launch.py")
    spec = importlib.util.spec_from_file_location("global_launch", global_main_launch)
    if spec is None:
        raise ImportError(f"Couldn't import global_launch module from {global_main_launch}")
    module = importlib.util.module_from_spec(spec)  # type: ignore[arg-type] # spec is not None
    spec.loader.exec_module(module)  # type: ignore[union-attr] # spec is not None
    global_launch_arguments = module.GLOBAL_LAUNCH_ARGUMENTS
    global_environment_vars = module.ENVIRONMENT_VARIABLES
    return global_launch_arguments, global_environment_vars


def setup_launch(context: LaunchContext) -> List[Node]:
    """Collects launch descriptions that describe the system behavior in the `local_pathfinding`
    package.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        List[Node]: Nodes to launch.
    """
    mode = LaunchConfiguration("mode").perform(context)
    on_water_mock_ais = LaunchConfiguration("on_water_mock_ais").perform(context)
    launch_description_entities = []
    launch_description_entities.append(get_navigate_node_description(context))
    if mode == "production":
        launch_description_entities.append(get_global_path_node_description(context))
        if on_water_mock_ais:
            launch_description_entities.append(get_mock_ais_node_description(context))

    elif mode == "development":
        launch_description_entities.append(get_mock_global_path_node_description(context))
        launch_description_entities.append(get_mock_wind_sensor_node_description(context))
        launch_description_entities.append(get_mock_ais_node_description(context))
        launch_description_entities.append(get_mock_gps_node_description(context))
        launch_description_entities.append(get_navigate_observer_node_description(context))

    return launch_description_entities


def get_navigate_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the navigate_main node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the navigate_main node.
    """
    node_name = "navigate_main"
    mode = LaunchConfiguration("mode").perform(context)
    inline_params = {"mode": mode}

    if mode == "development":
        inline_params["test_plan"] = LaunchConfiguration("test_plan").perform(context)

    ros_parameters = [
        LaunchConfiguration("config").perform(context),
        inline_params,
    ]

    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="navigate",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_global_path_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the global_path node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the global_path node.
    """
    node_name = "global_path"
    ros_parameters = [
        LaunchConfiguration("config").perform(context),
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="global_path",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_global_path_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mgp_main node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the mgp_main node.
    """
    node_name = "mock_global_path"
    test_plan = LaunchConfiguration("test_plan").perform(context)
    # Pass the shared params file plus inline mode/test_plan dict so only
    # the first entry is treated as a parameter file path.
    ros_parameters = [
        LaunchConfiguration("config").perform(context),
        {"test_plan:=": test_plan},
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="mock_global_path",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_ais_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mock ais node."""
    node_name = "mock_ais"
    test_plan = LaunchConfiguration("test_plan").perform(context)
    ros_parameters = [
        LaunchConfiguration("config").perform(context),
        {"test_plan": test_plan},
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="mock_ais",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_wind_sensor_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mock wind sensor node"""
    node_name = "mock_wind_sensor"
    test_plan = LaunchConfiguration("test_plan").perform(context)
    ros_parameters = [
        LaunchConfiguration("config").perform(context),
        {"test_plan": test_plan},
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="mock_wind_sensor",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_mock_gps_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the mock gps node"""
    node_name = "mock_gps"
    test_plan = LaunchConfiguration("test_plan").perform(context)

    use_gps_noise = LaunchConfiguration("use_gps_noise").perform(context)
    use_gps_noise_bool = use_gps_noise.lower() in ["true", "1", "yes"]

    use_ocean_drift = LaunchConfiguration("use_ocean_drift").perform(context)
    use_ocean_drift_bool = use_ocean_drift.lower() in ["true", "1", "yes"]

    use_drift_randomization = LaunchConfiguration("use_drift_randomization").perform(context)
    use_drift_randomization_bool = use_drift_randomization.lower() in ["true", "1", "yes"]

    ocean_drift_speed_kmph = float(LaunchConfiguration("ocean_drift_speed_kmph").perform(context))
    ocean_drift_dir_deg = float(LaunchConfiguration("ocean_drift_dir_deg").perform(context))
    ocean_drift_accel_kmph2 = float(
        LaunchConfiguration("ocean_drift_accel_kmph2").perform(context)
    )
    ros_parameters = [
        LaunchConfiguration("config").perform(context),
        {
            "test_plan": test_plan,
            "use_gps_noise": use_gps_noise_bool,
            "use_ocean_drift": use_ocean_drift_bool,
            "use_drift_randomization": use_drift_randomization_bool,
            "ocean_drift_speed_kmph": ocean_drift_speed_kmph,
            "ocean_drift_dir_deg": ocean_drift_dir_deg,
            "ocean_drift_accel_kmph2": ocean_drift_accel_kmph2,
        },
    ]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="mock_gps",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node


def get_navigate_observer_node_description(context: LaunchContext) -> Node:
    """Gets the launch description for the navigate_observer node.

    Args:
        context (LaunchContext): The current launch context.

    Returns:
        Node: The node object that launches the navigate_observer node.
    """
    node_name = "navigate_observer"
    ros_parameters = [LaunchConfiguration("config").perform(context)]
    ros_arguments: List[SomeSubstitutionsType] = [
        "--log-level",
        [f"{node_name}:=", LaunchConfiguration("log_level")],
    ]

    node = Node(
        package=PACKAGE_NAME,
        executable="navigate_observer",
        name=node_name,
        output="screen",
        emulate_tty=True,
        parameters=ros_parameters,
        ros_arguments=ros_arguments,
    )

    return node
