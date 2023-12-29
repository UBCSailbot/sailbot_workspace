import argparse
import os
import signal
import subprocess
import sys
import time
from enum import Enum

import rclpy
import yaml
from gen_dtypes import gen_dtypes
from rclpy.node import Node

# Need to generate dtypes.py before importing it
gen_dtypes()

from dtypes import get_dtype  # noqa: E402


class ROSPkg(Enum):
    boat_simulator = 0
    controller = 1
    local_pathfinding = 2
    network_systems = 3


# NAMESPACE = "INTEGRATION_TEST_NODE"
TIMEOUT_S = 30  # Number of seconds that the test has to run
ROS_LAUNCH_CMD = "ros2 launch {} main_launch.py mode:=development"
ROS_WORKSPACE_PATH = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbotworkspace")
ROS_PACKAGES_DIR = os.path.join(
    os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace"), "src"
)
# ROS_PACKAGES = ["boat_simulator", "controller", "local_pathfinding", "network_systems"]
ROS_PACKAGES = [pkg.name for pkg in ROSPkg]
ROS_PACKAGE_CONFIG_DIRS = {
    ROSPkg.boat_simulator: os.path.join(
        ROS_PACKAGES_DIR, ROSPkg.boat_simulator.name, "src/config"
    ),
    ROSPkg.controller: os.path.join(ROS_PACKAGES_DIR, ROSPkg.controller.name, "src/config"),
    ROSPkg.local_pathfinding: os.path.join(
        ROS_PACKAGES_DIR, ROSPkg.local_pathfinding.name, "src/config"
    ),
    ROSPkg.network_systems: os.path.join(
        ROS_PACKAGES_DIR, ROSPkg.network_systems.name, "src/config"
    ),
}


def get_ros_launch_cmd(package_name: str, launch_config_files: list[str]):
    launch_cmd = ROS_LAUNCH_CMD.format(package_name)

    if launch_config_files is not None:

        def convert_to_abs_path(config_file_path: str):
            pkg = ROSPkg[package_name]
            pkg_config_dir = ROS_PACKAGE_CONFIG_DIRS[pkg]
            return os.path.join(pkg_config_dir, config_file_path)

        launch_config_files_abs_path = [convert_to_abs_path(file) for file in launch_config_files]
        config_files_str = ",".join(launch_config_files_abs_path)
        launch_cmd += " config:={}".format(config_files_str)

    print(launch_cmd)
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

        inputs = params["inputs"]
        expected_outputs = params["expected_outputs"]
        procs = launch_modules(params["required_packages"])

        return procs, inputs, expected_outputs


def is_builtin_type(x):
    return x.__class__.__module__ == "builtins"


def set_ros_input(input: dict):
    msg, _ = get_dtype(input["dtype"])

    if is_builtin_type(msg):
        return msg(input["val"])

    for field in input:
        if field != "dtype":
            setattr(msg, field, set_ros_input(input[field]))
    return msg


class IntegrationTest:
    def __init__(self, config_file: str):
        procs, inputs, expected_outputs = setup_test(config_file)
        self.__procs = procs

        try:
            self.__ros_inputs: list[dict] = []
            self.__http_inputs: list[dict] = []

            self.__set_inputs(inputs)
        except:  # noqa: E722
            self.finish()

    def __set_inputs(self, inputs: list[dict]):
        for input in inputs:
            if input["type"] == "ROS":
                new_input_dict = {"topic": "", "msg_type": None, "msg": None}
                new_input_dict["topic"] = input["name"]

                data = input["data"]
                msg, msg_type = get_dtype(data["dtype"])
                new_input_dict["msg_type"] = msg_type
                if is_builtin_type(msg):
                    # std_msgs need an extra "data" field so we have to do some extra processing
                    val = msg(data["val"])
                    msg = msg_type()  # Change built-in type to std_msg type
                    msg.data = val
                else:
                    for field in data:
                        if field != "dtype":
                            setattr(msg, field, set_ros_input(data[field]))

                new_input_dict["msg"] = msg
                self.__ros_inputs.append(new_input_dict)

            elif input["type"] == "HTTP":
                # TODO: IMPLEMENT
                raise NotImplementedError("HTTP support is a WIP")
            else:
                raise KeyError("Invalid input type: {}".format(input["type"]))

    def ros_inputs(self):
        return self.__ros_inputs

    def http_inputs(self):
        return self.__http_inputs

    def finish(self):
        stop_modules(self.__procs)


class IntegrationTestNode(Node):
    def __init__(self, config_file: str):
        super().__init__("integration_test_node")

        self.__test_inst = IntegrationTest(config_file)

        try:
            self.__ros_inputs: list[dict] = []
            for ros_input in self.__test_inst.ros_inputs():
                pub = self.create_publisher(
                    msg_type=ros_input["msg_type"],
                    topic=ros_input["topic"],
                    qos_profile=10,  # placeholder
                )
                self.__ros_inputs.append(
                    {
                        "pub": pub,
                        "msg": ros_input["msg"],
                    }
                )

            # TODO: HTTP

            self.send_inputs()

            self.timeout = self.create_timer(TIMEOUT_S, self.__timeout_cb)
        except:  # noqa: E722
            self.__test_inst.finish()

    def __pub_ros(self):
        for ros_input in self.__ros_inputs:
            pub = ros_input["pub"]
            msg = ros_input["msg"]
            pub.publish(msg)
            self.get_logger().info(
                'Published to topic: "{topic}", with msg: "{msg}"'.format(topic=pub.topic, msg=msg)
            )

    def __timeout_cb(self):
        print("TIMEOUT")
        self.__test_inst.finish()

    def send_inputs(self):
        self.__pub_ros()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", required=True, metavar="path", dest="config_file")

    rclpy.init(args=args)
    node = IntegrationTestNode(parser.parse_args().config_file)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
