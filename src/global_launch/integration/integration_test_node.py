import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from gen_dtypes import gen_dtypes
from rclpy.node import Node

gen_dtypes()

# Need to generate the file before importing it
from dtypes import get_dtype  # noqa: E402

# NAMESPACE = "INTEGRATION_TEST_NODE"
TIMEOUT_S = 5  # Number of seconds that the test has to run
ROS_LAUNCH_CMD = "ros2 launch {} main_launch.py"
ROS_PACKAGES = ["boat_simulator", "controller", "local_pathfinding", "network_systems"]
ROS_WORKSPACE_PATH = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbotworkspace")


def get_ros_launch_cmd(package_name: str, launch_config_files: list[str]):
    launch_cmd = ROS_LAUNCH_CMD.format(package_name)

    if launch_config_files is not None:
        config_files_str = "".join(launch_config_files)
        launch_cmd += " --config:={}".format(config_files_str)

    return launch_cmd


def launch_modules(packages: list[dict]):
    launch_cmds: list[str] = []
    for package in packages:
        if package["name"] in ROS_PACKAGES:
            launch_cmds.append(get_ros_launch_cmd(package["name"], package["configs"]))
        else:
            match package["name"]:
                case "virtual_iridium":
                    run_viridium_cmd = os.path.join(ROS_WORKSPACE_PATH, "run_virtual_iridium.sh")
                    launch_cmds.append(run_viridium_cmd)
                case "website":
                    pass
                case _:
                    sys.exit("Error, invalid package name: {}".format(package["name"]))

    procs: list[subprocess.Popen] = []
    for cmd in launch_cmds:
        procs.append(
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid,
                # Use os.setsid to set process group such that we can
                # cleanly kill all spawned processes at the end of the test(s)
            )
        )
    return procs


def stop_modules(process_list: list[subprocess.Popen]):
    for proc in process_list:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)


def setup_test(config_file: str):
    with open(config_file, "r") as config:
        params = yaml.safe_load(config)

        procs = launch_modules(params["required_packages"])
        inputs = params["inputs"]
        expected_outputs = params["expected_outputs"]

        # time.sleep(TIMEOUT_S)  # Give the inputs time to propagate through the ROS network
        # stop_modules(procs)
        return procs, inputs, expected_outputs


def set_ros_input(input: dict):
    def is_builtin_type(x):
        return x.__class__.__module__ == "builtins"

    msg = get_dtype(input["dtype"])

    if is_builtin_type(msg):
        return msg(input["val"])

    for field in input:
        if field != "dtype":
            setattr(msg, field, set_ros_input(input[field]))
    return msg


class IntegrationTest:
    def __init__(self, config_file: str):
        procs, inputs, expected_outputs = setup_test(config_file)
        self.procs = procs
        self.inputs = inputs
        self.expected_outputs = expected_outputs

    def send_inputs(self):
        for input in self.inputs:
            if input["type"] == "ROS":
                topic = input["name"]
                msg = get_dtype(input["dtype"])
                data = input["data"]
                for field in data:
                    # msg[field] = set_ros_input(data[field])
                    setattr(msg, field, set_ros_input(data[field]))
                print(msg)
            elif input["type"] == "HTTP":
                raise NotImplementedError("HTTP support is a WIP")
            else:
                raise KeyError("Invalid input type: {}".format(input["type"]))

    def finish(self):
        stop_modules(self.procs)


class IntegrationTestNode(Node):
    def __init__(self):
        super().__init__("integration_test_node")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", required=True, metavar="path", dest="config_file")
    args = parser.parse_args()
    test_inst = IntegrationTest(args.config_file)
    test_inst.send_inputs()

    time.sleep(TIMEOUT_S)  # Give the inputs time to propagate through the ROS network
    test_inst.finish()

    rclpy.init(args=args)
    node = IntegrationTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
