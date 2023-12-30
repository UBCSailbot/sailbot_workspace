import argparse
import builtins
import functools
import importlib
import os
import signal
import subprocess
import sys
import time
from enum import Enum
from typing import Tuple, Union

import gen_dtypes
import rclpy
import yaml
from rclpy.node import MsgType, Node

# Generate and import datatypes
gen_dtypes.gen_dtypes()
DTYPES_MOD = importlib.import_module("dtypes")


MIN_SETUP_DELAY_S = 1  # Minimum 1 second delay between setting up the test and sending inputs
TIMEOUT_S = 3  # Number of seconds that the test has to run


class ROSPkg(Enum):
    boat_simulator = 0
    controller = 1
    local_pathfinding = 2
    network_systems = 3


ROS_LAUNCH_CMD = "ros2 launch {} main_launch.py mode:=development"
ROS_WORKSPACE_PATH = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbotworkspace")
ROS_PACKAGES_DIR = os.path.join(
    os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace"), "src"
)
# ROS_PACKAGES = ["boat_simulator", "controller", "local_pathfinding", "network_systems"]
ROS_PACKAGES = [pkg.name for pkg in ROSPkg]
ROS_PACKAGE_CONFIG_DIRS = {
    ROSPkg.boat_simulator: os.path.join(ROS_PACKAGES_DIR, ROSPkg.boat_simulator.name, "config"),
    ROSPkg.controller: os.path.join(ROS_PACKAGES_DIR, ROSPkg.controller.name, "config"),
    ROSPkg.local_pathfinding: os.path.join(
        ROS_PACKAGES_DIR, ROSPkg.local_pathfinding.name, "config"
    ),
    ROSPkg.network_systems: os.path.join(ROS_PACKAGES_DIR, ROSPkg.network_systems.name, "config"),
}
NON_ROS_PACKAGES = ["virtual_iridium", "website"]


def get_ros_launch_cmd(ros_pkg_name: str, launch_config_files: list[str]):
    launch_cmd = ROS_LAUNCH_CMD.format(ros_pkg_name)

    if launch_config_files is not None:

        def convert_to_abs_path(config_file_path: str):
            ros_pkg = ROSPkg[ros_pkg_name]
            ros_pkg_config_dir = ROS_PACKAGE_CONFIG_DIRS[ros_pkg]
            return os.path.join(ros_pkg_config_dir, config_file_path)

        launch_config_files_abs_path = [convert_to_abs_path(file) for file in launch_config_files]
        config_files_str = ",".join(launch_config_files_abs_path)
        launch_cmd += " config:={}".format(config_files_str)

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
                # cleanly kill all spawned processes at the end of the test
            )
        )

    # Make sure we cleanup after receiving an interrupt
    def signal_handler(sig_no: int, frame):
        stop_modules(procs)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

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


def builtin_to_std_msg(builtin_type: builtins.type, msg_type: MsgType, val: str) -> MsgType:
    # std_msgs need an extra "data" field so we have to do some extra processing
    std_val = builtin_type(val)
    msg = msg_type()
    msg.data = std_val
    return msg


def set_ros_msg_field(input: dict) -> Union[builtins.type, MsgType]:
    msg, _ = DTYPES_MOD.get_dtype(input["dtype"])

    if is_builtin_type(msg):
        return msg(input["val"])

    msg = parse_ros_msg(msg, input)
    return msg


def parse_ros_msg(msg: MsgType, data: dict) -> MsgType:
    for key in data:
        if key != "dtype":
            val = data[key]
            if isinstance(val, builtins.list):
                sub_msg_list: list[MsgType] = []
                for i in val:
                    sub_msg, _ = parse_ros_data(i)
                    sub_msg_list.append(sub_msg)
                setattr(msg, key, sub_msg_list)
            else:
                setattr(msg, key, set_ros_msg_field(val))
    return msg


def parse_ros_data(data: dict) -> Tuple[Union[None, MsgType], MsgType]:
    msg, msg_type = DTYPES_MOD.get_dtype(data["dtype"])
    if "DONT_CARE" in data and data["DONT_CARE"] is True:
        return None, msg_type  # Still need to return msg_type so we can pub/sub to a topic
    if is_builtin_type(msg):
        msg = builtin_to_std_msg(msg, msg_type, data["val"])
    else:
        msg = parse_ros_msg(msg, data)

    return msg, msg_type


class IntegrationTest:
    def __init__(self, config_file: str):
        self.__setup_complete = False
        procs, inputs, expected_outputs = setup_test(config_file)
        self.__procs = procs

        try:
            self.__ros_inputs: list[dict] = []
            self.__ros_e_outputs: list[dict] = []
            self.__http_inputs: list[dict] = []

            self.__set_inputs(inputs)
            self.__set_expected_outputs(expected_outputs)
            self.__setup_complete = True
        except:  # noqa: E722
            self.finish()

    def __set_inputs(self, inputs: list[dict]):
        for input in inputs:
            if input["type"] == "ROS":
                new_input_dict = {"topic": "", "msg_type": None, "msg": None}
                new_input_dict["topic"] = input["name"]

                data = input["data"]
                msg, msg_type = parse_ros_data(data)

                new_input_dict["msg_type"] = msg_type
                new_input_dict["msg"] = msg
                self.__ros_inputs.append(new_input_dict)

            elif input["type"] == "HTTP":
                # TODO: IMPLEMENT
                raise NotImplementedError("HTTP support is a WIP")
            else:
                raise KeyError("Invalid input type: {}".format(input["type"]))

    def __set_expected_outputs(self, outputs: list[dict]):
        for output in outputs:
            if output["type"] == "ROS":
                new_output_dict = {"topic": "", "msg_type": None, "msg": None}
                new_output_dict["topic"] = output["name"]

                data = output["data"]
                msg, msg_type = parse_ros_data(data)

                new_output_dict["msg_type"] = msg_type
                new_output_dict["msg"] = msg
                self.__ros_e_outputs.append(new_output_dict)

            elif output["type"] == "HTTP":
                # TODO: IMPLEMENT
                raise NotImplementedError("HTTP support is a WIP")
            else:
                raise KeyError("Invalid output type: {}".format(output["type"]))

    def ros_inputs(self):
        return self.__ros_inputs

    def ros_expected_outputs(self):
        return self.__ros_e_outputs

    def http_inputs(self):
        return self.__http_inputs

    def setup_complete(self):
        return self.__setup_complete

    def finish(self):
        stop_modules(self.__procs)


class IntegrationTestNode(Node):
    def __init__(self, config_file: str):
        super().__init__("integration_test_node")

        self.__num_errs = 0
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

            self.output_monitor: dict[str, dict] = {}
            self.subs: list[Node.Subscriber] = []
            for e_ros_output in self.__test_inst.ros_expected_outputs():
                topic = e_ros_output["topic"]
                self.output_monitor[topic] = {}
                self.output_monitor[topic]["rcvd"] = False
                self.output_monitor[topic]["expected_msg"] = e_ros_output["msg"]
                sub = self.create_subscription(
                    msg_type=e_ros_output["msg_type"],
                    topic=topic,
                    # Fun fact, setting this callback without functools.partial() requires
                    # writing something atrocious like:
                    # callback=((lambda topic: (lambda msg: self.__sub_ros_cb(msg, topic)))(topic))
                    # Python variable scoping can be very funky
                    callback=functools.partial(self.__sub_ros_cb, topic=topic),
                    qos_profile=10,  # placeholder
                )
                self.subs.append(sub)

            # TODO: HTTP

            # IMPORTANT: MAKE SURE EXPECTED OUTPUTS ARE SETUP BEFORE SENDING INPUTS
            time.sleep(MIN_SETUP_DELAY_S)
            self.send_inputs()

            self.timeout = self.create_timer(TIMEOUT_S, self.__timeout_cb)
        except:  # noqa: E722
            self.__test_inst.finish()

    def __sub_ros_cb(self, rcvd_msg: MsgType, topic: str):
        self.output_monitor[topic]["rcvd"] = True
        expected_msg = self.output_monitor[topic]["expected_msg"]
        if expected_msg is not None:
            if rcvd_msg != expected_msg:
                self.__num_errs += 1
                err_msg = """
                    Mismatch between expected and received value on topic: \"{topic}\"!
                    Expected: \"{expected}\", but received: \"{rcvd}\"
                """.format(
                    topic=topic, expected=expected_msg, rcvd=rcvd_msg
                )
                self.get_logger().error(err_msg)

    def __pub_ros(self):
        for ros_input in self.__ros_inputs:
            pub = ros_input["pub"]
            msg = ros_input["msg"]
            pub.publish(msg)
            self.get_logger().info(
                'Published to topic: "{topic}", with msg: "{msg}"'.format(topic=pub.topic, msg=msg)
            )

    def __timeout_cb(self):
        self.__test_inst.finish()  # Stop tests

        if not self.__test_inst.setup_complete():
            self.__num_errs += 1
            self.get_logger().error(
                "Failed to setup tests! Integration test code is probably buggy!"
            )

        # Check that expected outputs were updated at all
        for topic in self.output_monitor:
            if self.output_monitor[topic]["rcvd"] is False:
                self.__num_errs += 1
                err_msg = 'No messages seen on expected output topic: "{topic}"!'.format(
                    topic=topic
                )
                self.get_logger().error(err_msg)

        if self.__num_errs > 0:
            self.get_logger().error("Errors in integration tests! View logs for details")
            sys.exit(-1)

        self.get_logger().info("Integration test successful!")
        sys.exit(0)

    def send_inputs(self):
        self.__pub_ros()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", required=True, metavar="path", dest="config_file")

    argparse_args = parser.parse_args()

    rclpy.init(args=args)
    node = IntegrationTestNode(argparse_args.config_file)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
