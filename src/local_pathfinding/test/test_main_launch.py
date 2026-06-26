import importlib.util
from pathlib import Path

import pytest
import yaml
from launch.launch_context import LaunchContext


def _load_main_launch_module():
    """Load main_launch.py by path because launch files are not package modules."""
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "main_launch.py"
    spec = importlib.util.spec_from_file_location("local_pathfinding_main_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("launch_value", "expected"),
    [
        ("True", True),
        ("true", True),
        ("False", False),
        ("false", False),
        ("", True),
    ],
)
def test_get_launch_bool_or_config(launch_value, expected, tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "test_node": {
                    "ros__parameters": {
                        "test_argument": True,
                    }
                }
            }
        )
    )
    context = LaunchContext()
    context.launch_configurations["test_argument"] = launch_value

    main_launch = _load_main_launch_module()

    assert (
        main_launch.get_launch_bool_or_config(
            context,
            "test_argument",
            str(config_path),
            "test_node",
        )
        is expected
    )


@pytest.mark.parametrize(
    ("config_file", "argument", "node_name", "expected"),
    [
        ("globals.yaml", "visualizer_mode", "navigate_main", False),
        ("globals.yaml", "on_water_mock_ais", "mock_ais", False),
        ("launch_globals.yaml", "visualizer_mode", "navigate_main", True),
        ("launch_globals.yaml", "on_water_mock_ais", "mock_ais", False),
        ("on_water_globals.yaml", "visualizer_mode", "navigate_main", True),
        ("on_water_globals.yaml", "on_water_mock_ais", "mock_ais", True),
    ],
)
def test_get_launch_bool_or_config_reads_values_from_repo_configs(
    config_file, argument, node_name, expected, tmp_path, monkeypatch
):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    config_path = (
        Path(__file__).resolve().parents[2] / "global_launch" / "config" / config_file
    )
    context = LaunchContext()
    context.launch_configurations[argument] = ""

    main_launch = _load_main_launch_module()

    assert (
        main_launch.get_launch_bool_or_config(
            context,
            argument,
            str(config_path),
            node_name,
        )
        is expected
    )
