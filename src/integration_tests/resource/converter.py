import datetime
import json
import os
import random as rand
import shutil
from typing import Any, Tuple

import yaml

ROOT_DIR = "/workspaces/sailbot_workspace/src/integration_tests/resource/Data"
NEW_DIR = "/workspaces/sailbot_workspace/src/integration_tests/resource/Converted Data"
WIDTH = 10
LENGTH = 20
TIMEOUT = 3
yaml.Dumper.ignore_aliases = lambda *args: True  # type: ignore

# Convert all the mesages in raye to ROS YAML format and JSON format


def ais_converter(file: str) -> Tuple[dict[str, Any], dict[str, Any]]:
    http_data = {"type": "HTTP", "name": "aisships", "data": {"dtype": "AISShips", "ships": []}}
    ros_data = {"type": "ROS", "name": "aisships", "data": {"dtype": "AISShips", "ships": []}}
    with open(os.path.join(subdir, file)) as f:
        data = json.load(f)
        ships_json = []
        ships_ros = []  # type: list[dict[str, Any]]
        for ship in data:
            ship_json = {
                "ID": ship[0],  # type: ignore
                "lat": ship[1],
                "lon": ship[2],
                "headingDegrees": ship[3],
                "speedKmph": ship[4],
                "rot": rand.uniform(-180, 180),
                "width": WIDTH,
                "length": LENGTH,
                "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }  # type: dict[str, float | int | str]

            ship_ros = [
                {
                    "dtype": "HelperAISShip",
                    "id": {"dtype": "uint32", "val": ship[0]},  # type: ignore
                    "lat_lon": {
                        "dtype": "HelperLatLon",
                        "latitude": {
                            "dtype": "float32",
                            "val": ship[1],
                        },
                        "longitude": {
                            "dtype": "float32",
                            "val": ship[2],
                        },
                    },
                    "cog": {
                        "dtype": "HelperHeading",
                        "heading": {
                            "dtype": "float32",
                            "val": ship[3],
                        },
                    },
                    "sog": {
                        "dtype": "HelperSpeed",
                        "speed": {
                            "dtype": "float32",
                            "val": ship[4],
                        },
                    },
                    "rot": {
                        "dtype": "HelperROT",
                        "rot": {
                            "dtype": "int8",
                            "val": int(rand.uniform(-128, 127)),
                        },
                    },
                    "width": {
                        "dtype": "HelperDimension",
                        "dimension": {
                            "dtype": "float32",
                            "val": WIDTH,
                        },
                    },
                    "length": {
                        "dtype": "HelperDimension",
                        "dimension": {
                            "dtype": "float32",
                            "val": LENGTH,
                        },
                    },
                },
            ]
            ships_json.append(ship_json)
            ships_ros = ships_ros + ship_ros
        http_data["data"]["ships"] = http_data["data"]["ships"] + ships_json  # type: ignore
        ros_data["data"]["ships"] = ros_data["data"]["ships"] + ships_ros  # type: ignore

    return http_data, ros_data


def gps_converter(file: str) -> dict[str, Any]:
    ros_data = {"type": "ROS", "name": "gps", "data": {"dtype": "GPS"}}
    with open(os.path.join(subdir, file)) as f:
        data = json.load(f)

        gps_ros = {
            "lat_lon": {
                "dtype": "HelperLatLon",
                "latitude": {
                    "dtype": "float32",
                    "val": data[1],
                },
                "longitude": {
                    "dtype": "float32",
                    "val": data[2],
                },
            },
            "heading": {
                "dtype": "HelperHeading",
                "heading": {
                    "dtype": "float32",
                    "val": data[3],
                },
            },
        }

        ros_data["data"].update(gps_ros)  # type: ignore

    return ros_data


def dump_http(data: dict[str, Any], subdir_name: str, msg_type: str) -> None:
    with open(os.path.join(subdir_name, msg_type + "_http_data.yaml"), "w") as f:
        yaml.dump(data, f, sort_keys=False)


def dump_ros(data: dict[str, Any], subdir_name: str, msg_type: str) -> None:
    with open(os.path.join(subdir_name, msg_type + "_ros_data.yaml"), "w") as f:
        yaml.dump(data, f, sort_keys=False)


def add_miscellaneous(inputs_outputs: dict[str, Any]) -> dict[str, Any]:
    return {
        "timeout": TIMEOUT,
        "required_packages": [
            {"name": "local_pathfinding", "configs": None},
            {
                "name": "network_systems",
                "configs": [
                    "all_disable.yaml",
                    "example/example_en.yaml",
                    "mock_ais/mock_ais_en_default.yaml",
                ],
            },
        ],
        "inputs": [inputs_outputs],
        "expected_outputs": [inputs_outputs],
    }


if __name__ == "__main__":
    for subdir, dir, files in os.walk(ROOT_DIR):
        subdir_name = os.path.join(NEW_DIR, subdir[len(ROOT_DIR) + 1 :])
        try:
            os.makedirs(subdir_name)
        except FileExistsError:
            shutil.rmtree(subdir_name)
            os.makedirs(subdir_name)

        for file in files:
            if file == "myAIS.json":
                http_data, ros_data = ais_converter(file)
                http_data = add_miscellaneous(http_data)
                ros_data = add_miscellaneous(ros_data)
                dump_http(http_data, subdir_name, msg_type="aisships")
                dump_ros(ros_data, subdir_name, msg_type="aisships")
            elif file == "myGPS.json":
                ros_data = gps_converter(file)
                ros_data = add_miscellaneous(ros_data)
                dump_ros(ros_data, subdir_name, msg_type="gps")
