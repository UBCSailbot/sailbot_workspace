#!/usr/bin/env python3

"""The ROS node for data collection."""
import inspect
import json
import os
import signal
import sys
from typing import Any, Type

import custom_interfaces.msg
import rclpy
import rclpy.utilities
import rosbag2_py
import rosidl_runtime_py
from rclpy.node import Node
from rclpy.serialization import serialize_message

import boat_simulator.common.constants as Constants


def shutdown_handler(signum: int, frame: Any) -> None:
    """Exit the program gracefully in response to a shutdown signal. This function is necessary for
    the ROS shutdown callback to be properly called.

    Args:
        signum (int): The signal number associated with the signal received.
        frame (Any): The current execution frame at the time the signal was received.
    """
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    if is_collection_enabled():
        try:
            # TODO Explore alternatives to using the signal library, such as ROS event handlers
            signal.signal(signal.SIGINT, shutdown_handler)
            rclpy.spin(node)
        finally:
            rclpy.shutdown()


def is_collection_enabled() -> bool:
    try:
        is_data_collection_enabled_index = (
            sys.argv.index(Constants.DATA_COLLECTION_CLI_ARG_NAME) + 1
        )
        is_data_collection_enabled = sys.argv[is_data_collection_enabled_index] == "true"
    except ValueError:
        is_data_collection_enabled = False
    return is_data_collection_enabled


class DataCollectionNode(Node):
    # TODO: Abstract the file writing operations and remove redundant checks for self.use_json and
    # self.use_bag.

    def __init__(self):
        super().__init__(node_name="data_collection_node")
        self.get_logger().debug("Initializing node...")
        self.__declare_ros_parameters()
        self.__init_msg_types_dict()
        self.__init_subscriptions()
        self.__init_io()
        self.__init_timer_callbacks()
        self.__init_shutdown_callbacks()
        self.get_logger().debug("Node initialization complete. Starting execution...")

    def __declare_ros_parameters(self):
        """Declares ROS parameters from the global configuration file that will be used in this
        node.
        """
        self.get_logger().debug("Declaring ROS parameters...")

        # TODO: Implement a CLI argument in the launch file to enable storing JSON in a human-
        # readable format as an option, ensuring default behavior prioritizes efficiency over human
        # readability in file writing.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("file_name", rclpy.Parameter.Type.STRING),
                ("qos_depth", rclpy.Parameter.Type.INTEGER),
                ("topics", rclpy.Parameter.Type.STRING_ARRAY),
                ("bag", rclpy.Parameter.Type.BOOL),
                ("json", rclpy.Parameter.Type.BOOL),
                ("write_period_sec", rclpy.Parameter.Type.DOUBLE),
            ],
        ),
        all_parameters = self._parameters
        for name, parameter in all_parameters.items():
            value_str = str(parameter.value)
            self.get_logger().debug(f"Got parameter {name} with value {value_str}")

    def __init_msg_types_dict(self):
        """Prepare dictionary of all msg types with key name and value class"""
        self.get_logger().debug("Initializing msg types dictionary...")

        self.__msg_types_dict = {}
        for name, cls in inspect.getmembers(custom_interfaces.msg, inspect.isclass):
            # Do not store hidden classes (which often start with "_" by convention)
            if not name.startswith("_"):
                self.__msg_types_dict[name] = cls

    def __init_subscriptions(self):
        """Initialize the subscriptions for this node. These subscriptions pertain to the topics
        from which data will be collected."""
        self.get_logger().debug("Initializing subscriptions...")

        self.__sub_topic_names = {}

        # Get topic names by extracting from all evenly indexed elements and all topic types from
        # oddly indexed elements since it is assumed that topic name and type alternate in the list
        # For example, [name1, type1, name2, type2, ...]
        topic_names = self.sub_topics[::2]
        topic_types = self.sub_topics[1::2]
        for topic_name, msg_type_name in zip(topic_names, topic_types):
            if msg_type_name not in self.__msg_types_dict:
                self.get_logger().error(
                    f"msg type {msg_type_name} does not exist. Please adjust the topics array in \
                        the boat simulator configuration file"
                )
                continue

            # Create subscription to each topic specified in the config file
            self.__sub_topic_names[topic_name] = msg_type_name
            self.create_subscription(
                msg_type=self.__msg_types_dict[msg_type_name],
                topic=topic_name,
                callback=lambda msg, tn=topic_name: self.__general_sub_callback(msg, tn),
                qos_profile=self.get_parameter("qos_depth").get_parameter_value().integer_value,
            )

    def __init_io(self):
        """Initialize JSON and ROS Bag files for writing and create the output directory."""

        # Create output directory for written data
        os.makedirs(Constants.DATA_COLLECTION_OUTPUT_DIR, exist_ok=True)

        # Initialize JSON and ROS bag files
        if self.use_json:
            self.__init_json_file()
        if self.use_bag:
            self.__init_ros_bag()

    def __init_json_file(self):
        """Initializes a JSON file for data logging."""
        self.get_logger().debug("Initializing json file...")

        self.__data_to_write = {}
        self.__json_index_counter = 0
        json_file_path = os.path.join(
            Constants.DATA_COLLECTION_OUTPUT_DIR, self.file_name + ".json"
        )

        if os.path.exists(json_file_path):
            self.get_logger().warn(
                f"JSON file with name {self.file_name} already exists. Overwriting old file..."
            )
            os.remove(json_file_path)

        # Open JSON file in append mode so continuous writes to end of file are possible
        self.__json_file = open(json_file_path, "a")

        # Open JSON array with left bracket (should be closed with right bracket upon shutdown)
        self.__json_file.write("[\n")

        # Initialize initial data to be None for each topic
        for topic_name in self.__sub_topic_names.keys():
            self.__data_to_write[topic_name] = None

    def __init_ros_bag(self):
        """Initializes ros bag for data logging.
        https://docs.ros.org/en/humble/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html
        """
        self.get_logger().debug("Initializing ros bag...")

        self.__writer = rosbag2_py.SequentialWriter()
        file_path = os.path.join(Constants.DATA_COLLECTION_OUTPUT_DIR, self.file_name)
        storage_options = rosbag2_py._storage.StorageOptions(uri=file_path, storage_id="sqlite3")
        converter_options = rosbag2_py._storage.ConverterOptions("", "")
        self.__writer.open(storage_options, converter_options)

        for topic_name, msg_type_name in self.__sub_topic_names.items():
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic_name,
                type=msg_type_name,
                serialization_format="cdr",
            )
            self.__writer.create_topic(topic_info)

    def __init_timer_callbacks(self):
        """Initializes timer callbacks of this node that are executed periodically."""
        self.get_logger().debug("Initializing timer callbacks...")
        self.create_timer(timer_period_sec=self.json_write_period, callback=self.__write_to_json)

    def __init_shutdown_callbacks(self):
        """Initializes shutdown callbacks of this node that are executed on shutdown."""
        self.get_logger().debug("Initializing shutdown callbacks...")
        self.context.on_shutdown(self.__shutdown_callback)

    # SUBSCRIPTION CALLBACKS
    def __general_sub_callback(self, msg: Type, topic_name: str):
        """General subscription callback triggered when subscribed topics publish new data. For
        JSON this function stores the message to later be written into the file by a timer
        callback, and for ros bag this writes the message to the bag.

        Args:
            msg (Type): The message published by the subscribed topic.
            topic_name (str): The name of the topic that published the message.
        """
        if self.use_json:
            msg_as_ord_dict = rosidl_runtime_py.message_to_ordereddict(msg)
            self.__data_to_write[topic_name] = msg_as_ord_dict

        if self.use_bag:
            self.__writer.write(
                topic_name, serialize_message(msg), self.get_clock().now().nanoseconds
            )

    # TIMER CALLBACKS
    def __write_to_json(self):
        """Write the most recent data to a JSON file if all subscribed topics have received at
        least one message."""

        # TODO: Handle the case where the subscribed topic is not launched to ensure data is
        # written to JSON.
        if not any(value is None for value in self.__data_to_write.values()):
            # Recorded time is assumed to be evenly spaced by a specified period
            self.__data_to_write["time"] = self.__json_index_counter * self.json_write_period

            # Create a Python dictionary and serialize it for writing
            item_to_write = {self.__json_index_counter: self.__data_to_write}
            json_string = json.dumps(item_to_write, indent=4)

            # The first entry should not have a prepended comma. All other entries should.
            if self.__json_index_counter > 0:
                json_string = ",\n" + json_string

            # Write the JSON string to the JSON file and increment the index counter for next write
            self.__json_file.write(json_string)
            self.__json_index_counter += 1

    # SHUTDOWN CALLBACKS
    def __shutdown_callback(self):
        """Shutdown callback to close JSON file and ros bag."""
        self.get_logger().debug("Closing the storage files...")

        # Close the JSON array and then terminate I/O with the JSON file gracefully.
        if self.use_json:
            self.__json_file.write("\n]")
            self.__json_file.close()

        # Stop ROS Bag I/O. Needs to be called after JSON close to prevent early exiting.
        if self.use_bag:
            self.__writer.close()

    @property
    def file_name(self):
        return self.get_parameter("file_name").get_parameter_value().string_value

    @property
    def sub_topics(self):
        return self.get_parameter("topics").get_parameter_value().string_array_value

    @property
    def use_bag(self):
        return self.get_parameter("bag").get_parameter_value().bool_value

    @property
    def use_json(self):
        return self.get_parameter("json").get_parameter_value().bool_value

    @property
    def json_write_period(self):
        return self.get_parameter("write_period_sec").get_parameter_value().double_value


if __name__ == "__main__":
    main()
