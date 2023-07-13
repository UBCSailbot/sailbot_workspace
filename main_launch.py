import os
from collections import namedtuple
from typing import List, Dict
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# TODO: Add the controller package when it is ready
ROS_PACKAGES_DIR = os.path.join(os.getenv("ROS_WORKSPACE"), "src")
PRODUCTION_ROS_PACKAGES = ["local_pathfinding", "network_systems"]
SIMULATION_ROS_PACKAGES = ["boat_simulator", "local_pathfinding", "network_systems"]
LAUNCH_FILENAME_ENDING = "_launch.py"

# Add launch arguments here and edit launch_setup() to use new argument(s)
# if needed for global setup.
# TODO: Pass global arguments to children launch files.
LaunchArgument = namedtuple("LaunchArgument", ["name", "default", "description"])
LAUNCH_ARGUMENTS = [
    LaunchArgument("mode", "production", "System mode. One of 'production', 'simulation'.")
]
LAUNCH_MODES = ["production", "simulation"]


def generate_launch_description() -> LaunchDescription:
    declared_launch_arguments_list = generate_launch_argument_declarations()
    launch_description = LaunchDescription([
        *declared_launch_arguments_list,
        OpaqueFunction(function=launch_setup)
    ])
    return launch_description


def generate_launch_argument_declarations() -> List[DeclareLaunchArgument]:
    declared_arguments = list()
    for argument in LAUNCH_ARGUMENTS:
        declared_argument = DeclareLaunchArgument(
            name=argument.name,
            default_value=argument.default,
            description=argument.description
        )
        declared_arguments.append(declared_argument)
    return declared_arguments


def launch_setup(context : LaunchContext, *args, **kwargs) -> List[IncludeLaunchDescription]:
    launch_args = parse_launch_args(context=context)
    ros_package_list = get_running_ros_packages(mode=launch_args["mode"])
    include_launch_descriptions = get_include_launch_descriptions(ros_packages=ros_package_list)
    return include_launch_descriptions


def parse_launch_args(context : LaunchContext) -> Dict[str, str]:
    parsed_args = dict()
    for argument in LAUNCH_ARGUMENTS:
        parsed_args[argument.name] = LaunchConfiguration(argument.name).perform(context)
    return parsed_args


def get_include_launch_descriptions(ros_packages : List[str]) -> List[IncludeLaunchDescription]:
    package_include_launch_descriptions = list()
    for package in ros_packages:
        launch_file_abs_path = os.path.join(ROS_PACKAGES_DIR, package, 'launch', package) +\
            LAUNCH_FILENAME_ENDING
        include_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_abs_path)
        )
        package_include_launch_descriptions.append(include_launch_description)
    return package_include_launch_descriptions


def get_running_ros_packages(mode : str) -> List[str]:
    match mode:
        case "production":
            return PRODUCTION_ROS_PACKAGES
        case "simulation":
            return SIMULATION_ROS_PACKAGES
        case _:
            raise ValueError("Invalid launch mode. Must be one of 'production', 'simulation'.")
