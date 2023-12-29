import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from dtypes.py import get_dtype
from rclpy.node import Node

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


def run_test(config_files: list[str]):
    for config_file in config_files:
        with open(config_file, "r") as config:
            params = yaml.safe_load(config)

            procs = launch_modules(params["required_packages"])

            time.sleep(TIMEOUT_S)  # Give the inputs time to propagate through the ROS network
            stop_modules(procs)


class IntegrationTestNode(Node):
    def __init__(self):
        super().__init__("integration_test_node")

        # self.declare_parameters(
        #     namespace=NAMESPACE,
        #     parameters=[
        #         ("")
        #     ]
        # )

    def set_ros_input(self, input):
        if isinstance(input, dict):
            msg = get_dtype(input["dtype"])
            for field in input:
                if field != "dtype":
                    msg[field] = self.set_ros_input(input[field])
            return msg
        else:
            return input

    def send_inputs(self):
        for input in self.inputs:
            if input["type"] == "ROS":
                topic = input["name"]
                msg = get_dtype(input["dtype"])
                data = input["data"]
                for field in data:
                    msg[field] = self.set_ros_input(data[field])
            else:
                pass


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", required=True, metavar="path", dest="config_files")
    args = parser.parse_args()
    run_test(args.config_files.split(","))

    rclpy.init(args=args)
    node = IntegrationTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
