import builtins
import functools
import json
import os
import signal
import subprocess
import sys
import time
import traceback
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple, Type, Union

import custom_interfaces.msg  # type: ignore
import rclpy  # type: ignore
import rclpy.node  # type: ignore
import std_msgs.msg  # type: ignore
import yaml
from rclpy.impl.rcutils_logger import RcutilsLogger  # type: ignore
from rclpy.node import MsgType, Node

MIN_SETUP_DELAY_S = 1  # Minimum 1 second delay between setting up the test and sending inputs
DEFAULT_TIMEOUT_SEC = 3  # Number of seconds that the test has to run

ROS_LAUNCH_CMD = "ros2 launch {} main_launch.py mode:=development"
ROS_WORKSPACE_PATH = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbotworkspace")
ROS_PACKAGES_DIR = os.path.join(
    os.getenv("ROS_WORKSPACE", default="/workspaces/sailbot_workspace"), "src"
)
ROS_PACKAGES = ["boat_simulator", "controller", "local_pathfinding", "network_systems"]
NON_ROS_PACKAGES = ["virtual_iridium", "website"]

# TODO: TestMsgType needs to encompass/inherit whatever type we use for HTTP messages
HTTP_MSG_PLACEHOLDER_TYPE = builtins.dict[str, Any]
TestMsgType = Union[rclpy.node.MsgType, HTTP_MSG_PLACEHOLDER_TYPE]

GLOBAL_PATH_API_NAME = "globalpath"
POST_URL = "http://localhost:8081/global-path"
PULL_URL = "http://localhost:3005/api/"


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


def get_ros_launch_cmd(ros_pkg_name: str, launch_config_files: list[str]) -> str:
    """Returns a command to launch a ROS package with specified config files

    Args:
        ros_pkg_name (str): A ROS package found in the ROS_PACKAGES constant
        launch_config_files (list[str]): A list of relative config file paths

    Raises:
        ValueError: If ros_pkg_name is not a valid ROS package

    Returns:
        str: A shell command to launch the desired ROS package with specified config files
    """

    if ros_pkg_name not in ROS_PACKAGES:
        raise ValueError(
            f"Given ros_pkg_name ({ros_pkg_name}) is not a valid ROS package ({ROS_PACKAGES})"
        )

    launch_cmd = ROS_LAUNCH_CMD.format(ros_pkg_name)

    if launch_config_files is not None:
        launch_config_files_abs_path = [
            os.path.join(ROS_PACKAGES_DIR, ros_pkg_name, "config", f) for f in launch_config_files
        ]
        config_files_str = ",".join(launch_config_files_abs_path)
        launch_cmd += f" config:={config_files_str}"

    return launch_cmd


@dataclass
class TestPlan:
    """Represents the KVPs of the required_packages field in testplan yaml files"""

    name: str
    configs: list[str]

    def __init__(self, pkg_dict: dict):
        """Initialize a TestPlan instance from a testplan required_packages entry dict

        Args:
            pkg_dict (dict): required_packages entry
        """
        self.name = pkg_dict["name"]
        self.configs = pkg_dict["configs"]


def launch_modules(packages: list[TestPlan]) -> list[subprocess.Popen]:
    """Launches specified modules in background processes. Also registers a signal handler to kill
    any spawned processes.

    Args:
        packages (list[TestPlan]): List of modules to launch with their respective config files

    Raises:
        NotImplementedError: If "website" package is specified. It's on the TODO list.
        ValueError: If there is an invalid package.

    Returns:
        list[subprocess.Popen]: The handles of all spawned subprocesses. These must be killed
        on exit.
    """
    launch_cmds: list[str] = []
    for pkg in packages:
        if pkg.name in ROS_PACKAGES:
            launch_cmds.append(get_ros_launch_cmd(pkg.name, pkg.configs))
        else:
            match pkg.name:
                case "virtual_iridium":
                    run_viridium_cmd = os.path.join(
                        ROS_WORKSPACE_PATH, "scripts", "run_virtual_iridium.sh"
                    )
                    launch_cmds.append(run_viridium_cmd)
                case "website":
                    raise NotImplementedError("Website not supported yet")
                case _:
                    raise ValueError(f"Invalid package name: {pkg.name}")

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

    def signal_handler(sig_no: int, frame) -> None:
        """Kill spawned processes on interrupt and exit

        Args:
            sig_no (int): Interrupt signal number - unused
            frame (something): unused
        """
        stop_modules(procs)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    return procs


def stop_modules(process_list: list[subprocess.Popen]) -> None:
    """Kill running modules

    Args:
        process_list (list[subprocess.Popen]): List of subprocess handles to kill
    """
    for proc in process_list:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)


def setup_test(testplan_file: str) -> Tuple[list[subprocess.Popen], list[dict], list[dict], int]:
    """Reads a test testplan file and sets up the test environment

    Args:
        testplan_file (str): Path to a .yaml test testplan file

    Returns:
        list[subprocess.Popen]: List of spawned subprocess handles
        list[dict]: inputs parsed from testplan_file
        list[dict]: expected outputs parsed from testplan_file
        int: test timeout in seconds
    """
    with open(testplan_file, "r") as testplan:
        params = yaml.safe_load(testplan)

        inputs = params["inputs"]
        expected_outputs = params["expected_outputs"]
        pkgs = [TestPlan(pkg_dict) for pkg_dict in params["required_packages"]]
        procs = launch_modules(pkgs)
        timeout_sec = params["timeout_sec"] if "timeout_sec" in params else DEFAULT_TIMEOUT_SEC

        return procs, inputs, expected_outputs, timeout_sec


def is_builtin_type(x: Any) -> bool:
    """Checks if an object is a builtin type

    Args:
        x (Any): object to check

    Returns:
        bool: True if x is a builtin type, False otherwise
    """
    return x.__class__.__module__ == "builtins"


def builtin_to_std_msg(
    builtin_type: builtins.type, msg_type: rclpy.node.MsgType, val: str
) -> rclpy.node.MsgType:
    """Convert a builtin type to a ROS std_msg type (rclpy.node.MsgType)

    Args:
        builtin_type (builtins.type): builtin type to convert
        msg_type (rclpy.node.MsgType): ROS type to convert to
        val (str): Value of the type

    Returns:
        rclpy.node.MsgType: Conversion of the givin builtin type to ROS type
    """
    # std_msgs need an extra "data" field so we have to do some extra processing
    std_val = builtin_type(val)
    # mypy gives on error for this line but it works :)
    msg = msg_type()  # type: ignore
    msg.data = std_val
    return msg


def get_ros_dtype(dtype: str) -> Tuple[Union[builtins.type, MsgType], Type[MsgType]]:
    """Given the dtype as a str, return the Python type object and identifier

    Args:
        dtype (str): dtype str from

    Raises:
        TypeError: If an invalid dtype is given

    Returns:
        Union[builtins.type, MsgType]: Type object
        Type[MsgType]: Type identifier
    """
    match dtype:
        # builtin types see "Type name" column in:
        # https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#field-types

        case "bool":
            return builtins.bool, std_msgs.msg.Bool

        case "byte":
            return builtins.bytes, std_msgs.msg.Byte

        case "char":
            return builtins.str, std_msgs.msg.String

        case "float32":
            return builtins.float, std_msgs.msg.Float32

        case "float64":
            return builtins.float, std_msgs.msg.Float64

        case "int8":
            return builtins.int, std_msgs.msg.Int8

        case "uint8":
            return builtins.int, std_msgs.msg.UInt8

        case "int16":
            return builtins.int, std_msgs.msg.Int16

        case "uint16":
            return builtins.int, std_msgs.msg.UInt16

        case "int32":
            return builtins.int, std_msgs.msg.Int32

        case "uint32":
            return builtins.int, std_msgs.msg.UInt32

        case "int64":
            return builtins.int, std_msgs.msg.Int64

        case "uint64":
            return builtins.int, std_msgs.msg.UInt64

        case "string":
            return builtins.str, std_msgs.msg.String

        case "wstring":
            return builtins.str, std_msgs.msg.String

        # custom_interfaces
        case _:
            try:
                return (
                    getattr(custom_interfaces.msg, dtype)(),
                    getattr(custom_interfaces.msg, dtype),
                )  # type: ignore
            except AttributeError:
                raise TypeError(f"INVALID TYPE: {dtype}")


def get_ros_msg_field_val(input_dict: dict) -> Union[builtins.type, rclpy.node.MsgType]:
    """Get the field value of some ROS message type

    Args:
        input_dict (dict): dictionary containing a "dtype" field and other fields.

    Returns:
        Union[builtins.type, rclpy.node.MsgType]: A value of type builtins.type or
            rclpy.node.MsgType depending on what is set in the testplan.
    """
    msg, _ = get_ros_dtype(input_dict["dtype"])

    if is_builtin_type(msg):
        return msg(input_dict["val"])

    msg = parse_ros_msg(msg, input_dict)
    return msg


def parse_ros_msg(msg: rclpy.node.MsgType, data: dict) -> rclpy.node.MsgType:
    """Parse a ROS data field dictionary. Use this function to parse fields specified UNDER the
    "data" category, but no the "data" category itself.

    Args:
        msg (rclpy.node.MsgType): ROS msg type of the data
        data (dict): dictionary containing data subfields

    Returns:
        rclpy.node.MsgType: msg object with fields assigned
    """
    for key in data:
        if key != "dtype":
            val = data[key]
            if isinstance(val, builtins.list):
                sub_msg_list: list[Union[rclpy.node.MsgType, None]] = []
                for i in val:
                    sub_msg, _ = parse_ros_data(i)
                    sub_msg_list.append(sub_msg)
                setattr(msg, key, sub_msg_list)
            else:
                setattr(msg, key, get_ros_msg_field_val(val))
    return msg


def parse_ros_data(data: dict) -> Tuple[Union[None, rclpy.node.MsgType], rclpy.node.MsgType]:
    """Parse a data dictionary "data" field knowing only that it is ROS data. Use this function if
    dtype is currently unknown.

    Args:
        data (dict): dictionary containing ROS data

    Returns:
        Union[None, rclpy.node.MsgType]: ROS msg type OBJECT or None if DONT_CARE is specified
        rclpy.node.MsgType: ROS msg type IDENTIFIER
    """
    msg, msg_type = get_ros_dtype(data["dtype"])
    if "DONT_CARE" in data and data["DONT_CARE"] is True:
        return None, msg_type  # Still need to return msg_type so we can pub/sub to a topic
    if is_builtin_type(msg):
        msg = builtin_to_std_msg(msg, msg_type, data["val"])
    else:
        msg = parse_ros_msg(msg, data)

    return msg, msg_type


def parse_http_data(
    data: dict,
) -> Tuple[Union[None, HTTP_MSG_PLACEHOLDER_TYPE], HTTP_MSG_PLACEHOLDER_TYPE]:
    """Parses

    Args:
        data (dict): Stores the expected HTTP output

    Returns:
        Tuple[Union[None, HTTP_MSG_PLACEHOLDER_TYPE], HTTP_MSG_PLACEHOLDER_TYPE]: _description_
    """

    msg_type = data["dtype"]
    data.pop("dtype")

    return data, msg_type


@dataclass
class IOEntry:
    """Represents IO data"""

    name: str  # ROS topic or HTTP target
    msg_type: Type[TestMsgType]
    msg: Optional[TestMsgType]


class IntegrationTestSequence:
    """Class that defines a test sequence/flow"""

    def __init__(self, testplan_file: str):
        """Initializes a test sequences

        Args:
            config_file (str): Path to a test .yaml testplan file
        """
        procs, inputs, expected_outputs, timeout = setup_test(testplan_file)
        self.__procs = procs
        self.__timeout = timeout

        self.__ros_inputs: list[IOEntry] = []
        self.__ros_e_outputs: list[IOEntry] = []
        self.__http_inputs: list[IOEntry] = []
        self.__http_e_outputs: list[IOEntry] = []

        try:
            self.__set_inputs(inputs)
            self.__set_expected_outputs(expected_outputs)
        except Exception as e:
            self.finish()
            raise e

    def __set_inputs(self, inputs: list[dict]):
        """Prepare inputs that will drive the test

        Args:
            inputs (list[dict]): list of inputs

        Raises:
            NotImplementedError: GlobalPath messages are not yet supported
            KeyError: If an invalid input type is given. Valid input types are ROS and HTTP.
        """
        for input_dict in inputs:
            if input_dict["type"] == "ROS":
                data = input_dict["data"]
                msg, msg_type = parse_ros_data(data)  # type: ignore

                new_input = IOEntry(name=input_dict["name"], msg_type=msg_type, msg=msg)
                self.__ros_inputs.append(new_input)

            elif input_dict["type"] == "HTTP":
                data = input_dict["data"]
                if GLOBAL_PATH_API_NAME == input_dict["name"]:
                    # This is a GlobalPath message
                    msg, msg_type = parse_http_data(data)  # type: ignore
                    new_input = IOEntry(name=input_dict["name"], msg_type=msg_type, msg=msg)
                else:
                    msg, msg_type = parse_ros_data(data)  # type: ignore
                    new_input = IOEntry(name=input_dict["name"], msg_type=msg_type, msg=msg)

                self.__http_inputs.append(new_input)

            else:
                raise KeyError(f"Invalid input type: {input_dict['type']}")

    def __set_expected_outputs(self, outputs: list[dict]):
        """Prepare the test's expected outputs

        Args:
            outputs (list[dict]): list of expected outputs

        Raises:
            NotImplementedError: If an output of type HTTP is given. It's on the TODO list :)
            KeyError: If an invalid output type is given. Valid input types are ROS and HTTP.
        """
        for output in outputs:
            if output["type"] == "ROS":
                data = output["data"]
                msg, msg_type = parse_ros_data(data)  # type: ignore

                new_output = IOEntry(name=output["name"], msg_type=msg_type, msg=msg)
                self.__ros_e_outputs.append(new_output)

            elif output["type"] == "HTTP":
                data = output["data"]

                msg, msg_type = parse_http_data(data)  # type: ignore
                new_output = IOEntry(name=output["name"], msg_type=msg_type, msg=msg)
                self.__http_e_outputs.append(new_output)

            else:
                raise KeyError(f"Invalid output type: {output['type']}")

    def timeout_sec(self) -> int:
        """Get test timeout

        Returns:
            int: timeout value in seconds
        """
        return self.__timeout

    def ros_inputs(self) -> list[IOEntry]:
        """Return ROS inputs

        Returns:
            list[IOEntry]: ROS inputs
        """
        return self.__ros_inputs

    def ros_expected_outputs(self) -> list[IOEntry]:
        """Return expected ROS outputs

        Returns:
            list[IOEntry]: Expected ROS outputs
        """
        return self.__ros_e_outputs

    def http_inputs(self) -> list[IOEntry]:
        """Return HTTP inputs

        Returns:
            list[IOEntry]: HTTP inputs
        """
        return self.__http_inputs

    def http_expected_outputs(self) -> list[IOEntry]:
        """Return expected HTTP outputs

        Returns:
            list[IOEntry]: Expected HTTP outputs
        """
        return self.__http_e_outputs

    def finish(self) -> None:
        """Finish test sequence and cleanup"""
        stop_modules(self.__procs)


@dataclass
class MonitorEntry:
    """Represents entries in the Monitor class."""

    expected_msg: Union[TestMsgType, None]  # Accepts None types as sometimes we just care that
    # *something* is being output.
    rcvd_msgs: list[TestMsgType] = field(default_factory=list[TestMsgType])


class Monitor:
    """IO monitor that watches the data that passes through given interfaces. This data is saved"""

    def __init__(self) -> None:
        """Initialize a monitor instance"""
        self.__monitor: dict[str, MonitorEntry] = {}

    def register(self, name: str, expected_msg: Union[TestMsgType, None]) -> None:
        """Register an IO event to keep track of

        Args:
            name (str): Name of the event (ROS topic or HTTP target)
            expected_msg (TestMsgType | None): Event message type
        """
        self.__monitor[name] = MonitorEntry(expected_msg=expected_msg)

    def on_new_msg(self, name: str, rcvd_msg: TestMsgType) -> None:
        """Update the monitor with the message of a tracked IO event

        Args:
            name (str): Name of the event being tracked (ROS topic or HTTP target)
            rcvd_msg (TestMsgType): Message that was received at the event

        Raises:
            KeyError: If this function is called on an unregistered event
        """
        if name not in self.__monitor:
            raise KeyError(f"Tried to update Monitor with unregistered IO from {name}!")
        self.__monitor[name].rcvd_msgs.append(rcvd_msg)

    def evaluate(self, logger: RcutilsLogger) -> Tuple[int, int]:
        """Evaluate the results of all tracked events

        Args:
            logger (RcutilsLogger): ROS logger instance to use

        Returns:
            int: Number of failures
            int: Number of warnings
        """

        num_fails = 0
        num_warn = 0
        for name in self.__monitor:
            entry = self.__monitor[name]
            num_rcvd = len(entry.rcvd_msgs)

            if num_rcvd == 0:
                logger.error(f"No messages seen on: {name}!")
                num_fails += 1
            elif entry.expected_msg is not None:
                num_matches = entry.rcvd_msgs.count(entry.expected_msg)
                if num_matches == 0:
                    logger.error(
                        f"""
                                 No matching messages for: {name}!
                                 Expected: {entry.expected_msg}
                                 Received: {entry.rcvd_msgs}
                                 """
                    )
                    num_fails += 1
                elif num_matches < num_rcvd:
                    logger.warn(
                        f""" Partial matching message(s) for: {name}
                        Number of matching messages:       {num_matches}
                        Total number of received messages: {num_rcvd}
                        Expected message: {entry.expected_msg}
                        """
                    )
                    num_warn += 1
        return num_fails, num_warn


@dataclass
class ROSInputEntry:
    """Represents ROS publisher and msg pairs"""

    pub: rclpy.node.Publisher
    msg: rclpy.node.MsgType


class IntegrationTestNode(Node):
    """Node that connects integration tests to the ROS network"""

    def __init__(self) -> None:
        """Initialize the node

        Args:
            testplan_file (str): Path to test testplan .yaml file
        """
        super().__init__("integration_test_node")

        self.declare_parameter("testplan", rclpy.Parameter.Type.STRING)

        testplan_file = self.get_parameter("testplan").get_parameter_value().string_value

        self.__http_outputs: list[dict[str, Any]] = []
        self.__web_fail = 0

        try:
            self.__test_inst = IntegrationTestSequence(testplan_file)
            try:
                self.__ros_inputs: list[ROSInputEntry] = []
                for ros_input in self.__test_inst.ros_inputs():
                    pub = self.create_publisher(
                        msg_type=ros_input.msg_type,
                        topic=ros_input.name,
                        qos_profile=10,  # placeholder
                    )
                    self.__ros_inputs.append(ROSInputEntry(pub=pub, msg=ros_input.msg))

                self.__monitor = Monitor()
                self.__ros_subs: list[Node.Subscriber] = []
                for e_ros_output in self.__test_inst.ros_expected_outputs():
                    topic = e_ros_output.name
                    self.__monitor.register(topic, e_ros_output.msg)
                    sub = self.create_subscription(
                        msg_type=e_ros_output.msg_type,
                        topic=topic,
                        # Fun fact, setting this callback without functools.partial() requires
                        # writing something atrocious like:
                        # callback=
                        #     ((lambda topic: (lambda msg: self.__sub_ros_cb(msg, topic)))(topic))
                        # Python variable scoping can be very funky
                        callback=functools.partial(self.__sub_ros_cb, topic=topic),
                        qos_profile=10,  # placeholder
                    )
                    self.__ros_subs.append(sub)

                self.__http_inputs: list[ROSInputEntry] = []
                self.__global_path_pub = False
                for http_input in self.__test_inst.http_inputs():
                    if http_input.name != GLOBAL_PATH_API_NAME:
                        pub = self.create_publisher(
                            msg_type=http_input.msg_type,
                            topic=http_input.name,
                            qos_profile=10,  # change
                        )
                        self.__http_inputs.append(ROSInputEntry(pub=pub, msg=http_input.msg))
                    else:
                        # This is a GlobalPath message
                        self.__global_path_pub = self.pub_global_path(http_input.msg)

                        if self.__global_path_pub:
                            self.get_logger().info("Published GlobalPath to database")
                        else:
                            self.__web_fail = 1

                # IMPORTANT: MAKE SURE EXPECTED OUTPUTS ARE SETUP BEFORE SENDING INPUTS
                time.sleep(MIN_SETUP_DELAY_S)
                self.drive_inputs()

                self.timeout = self.create_timer(
                    self.__test_inst.timeout_sec(), self.__timeout_cb()  # type: ignore
                )
            except Exception as e:
                # At this point, the test instance has successfully started all package processes.
                # This except block is a failsafe to kill the processes in case anything crashes
                # the program
                self.__test_inst.finish()
                raise e
        except:  # noqa: 402
            self.get_logger().error(
                f"Failed to setup tests! Exception occured:\n{traceback.format_exc()}"
            )
            sys.exit(-1)

    def __sub_ros_cb(self, rcvd_msg: rclpy.node.MsgType, topic: str) -> None:
        """Callback to be executed when a subscribed ROS topic gets new data

        Args:
            rcvd_msg (rclpy.node.MsgType): data received at the subscribed topic
            topic (str): subscribed topic
        """
        self.__monitor.on_new_msg(topic, rcvd_msg)  # type: ignore # type hinting struggles

    def __pub_ros(self) -> None:
        """Publish to all registered ROS input topics"""
        for ros_input in self.__ros_inputs:
            pub = ros_input.pub
            msg = ros_input.msg
            pub.publish(msg)
            self.get_logger().info(f'Published to topic: "{pub.topic}", with msg: "{msg}"')

    def __pub_http(self) -> None:
        """Publish to all registered ROS input topics that interact with HTTP endpoints"""
        for web_input in self.__http_inputs:
            pub = web_input.pub
            msg = web_input.msg
            pub.publish(msg)
            self.get_logger().info(f'Published to topic: "{pub.topic}", with msg: "{msg}"')

    def pub_global_path(self, msg: Union[dict[str, Any], None]) -> bool:
        """Publish to the GlobalPath HTTP endpoint

        Args:
            msg (HTTP_MSG_PLACEHOLDER_TYPE): Message to publish
        """
        data = json.dumps(msg).encode("utf8")

        try:
            urllib.request.urlopen(
                POST_URL,
                data,
            )
            return True
        except urllib.error.URLError as e:
            self.get_logger().error(
                f"Failed to publish Global path to the remote transceiver: {e}"
            )
            return False
        except urllib.error.HTTPError as e:
            self.get_logger().error(
                f"Failed to publish Global path to the remote transceiver: {e}"
            )
            return False

    def http_evaluate(self, http_outputs: list[dict[str, Any]]) -> int:

        num_fail = 0

        for count, http_e_output in enumerate(self.__test_inst.http_expected_outputs()):
            topic = http_e_output.name
            data = http_outputs[count]

            if data != http_e_output.msg:
                self._logger.error(
                    f"HTTP output from {topic} does not match expected output: {data}"
                )
                num_fail += 1

        return num_fail

    def get_http_outputs(self) -> int:

        num_fail = 0

        for e_http_output in self.__test_inst.http_expected_outputs():
            topic = e_http_output.name
            try:
                contents = urllib.request.urlopen(PULL_URL + topic)
            except urllib.error.HTTPError as e:
                self._logger.error(f"HTTPError: {e}")
                return False

            data_dict = json.load(contents)
            try:
                data = (data_dict["data"])[0]
            except IndexError:
                self._logger.error(f"No data found for {topic}")
                return False

            data.pop(
                "timestamp"
            )  # Remove timestamp from data as it is not relevant for comparison
            self.__http_outputs.append(data)

        num_fail = self.http_evaluate(self.__http_outputs)

        return num_fail

    def __timeout_cb(self) -> None:
        """Callback for when the test times out. Stops all test processes, evaluates correctness,
        and exits
        """
        self.__test_inst.finish()  # Stop tests

        num_fail, num_warn = self.__monitor.evaluate(self.get_logger())
        num_fail += self.__web_fail + self.get_http_outputs()
        if num_warn > 0:
            self.get_logger().warn(
                (
                    f"Test finished with {num_warn} warnings. "
                    "Please check logs to verify that they are OK."
                )
            )

        if num_fail > 0:
            self.get_logger().error(
                f"Test finished with {num_fail} failures! Check logs for details."
            )
            sys.exit(-1)

        self.get_logger().info("Integration test finished successfully!")
        sys.exit(0)

    def drive_inputs(self) -> None:
        """Drive all registered test inputs"""
        self.__pub_ros()
        # TODO: add HTTP
        self.__pub_http()


if __name__ == "__main__":
    main()
