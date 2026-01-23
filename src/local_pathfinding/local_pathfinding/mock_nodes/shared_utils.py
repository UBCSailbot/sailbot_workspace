import os

import custom_interfaces.msg as ci
import yaml

MEAN_SPEED = ci.HelperSpeed(speed=15.0)  # mean boat speed in kmph,
START_POINT = ci.HelperLatLon(
    latitude=49.28, longitude=-123.185032
)  # Starting location of the mock
START_HEADING = ci.HelperHeading(heading=-90.0)  # in degrees, heading of the boat
TW_SPEED_KMPH = 10.0
TW_DIRECTION_DEG = 90

"""
For further tests:
START_POINT = ci.HelperLatLon(latitude=49.308157, longitude=-123.244801)
Change last line of mock_global_path.csv to: 49.289686,-123.195877
"""


def validate_tw_dir_deg(value: int) -> None:
    """
    Validate direction is in (-180, 180].
    """
    if not (-180 < value <= 180):
        raise ValueError(f"tw_dir_deg must be in (-180, 180]; got {value}")


def read_test_plan_file(file_path: str) -> dict:
    """Read test plan file and return its contents as a dictionary.

    Accepts either an absolute path or a test plan name like "basic.yaml".
    Relative paths are resolved against the local_pathfinding test_plans
    directory inside the current ROS workspace.

    Args:
        file_path (str): Path or name of the test plan file.
    Returns:
        dict: Contents of the test plan file.
    """

    # Resolve relative names (e.g. "basic.yaml") to the default test_plans dir
    if not os.path.isabs(file_path):
        ros_workspace = os.getenv("ROS_WORKSPACE", "/workspaces/sailbot_workspace")
        default_dir = os.path.join(
            ros_workspace,
            "src",
            "local_pathfinding",
            "test_plans",
        )
        file_path = os.path.join(default_dir, file_path)

    _, ext = os.path.splitext(file_path)

    if ext.lower() in [".yaml", ".yml"]:
        with open(file_path, "r") as file:
            data = yaml.safe_load(file)
        return data
    else:
        raise ValueError(f"Unsupported test plan file extension: {ext}")
