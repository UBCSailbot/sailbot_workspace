#!/usr/bin/env python3
import csv
import json
import sys
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


def flatten(value, prefix="", output=None):
    if output is None:
        output = {}

    if isinstance(value, Mapping):
        for key, item in value.items():
            name = f"{prefix}.{key}" if prefix else key
            flatten(item, name, output)
    elif isinstance(value, (list, tuple)):
        # Keep variable-length arrays readable in one CSV cell.
        output[prefix] = json.dumps(value, default=str)
    else:
        output[prefix] = value

    return output


bag_dir = Path(sys.argv[1])
output_dir = Path(sys.argv[2])
output_dir.mkdir(parents=True, exist_ok=True)

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    ),
)

topic_types = {
    topic.name: topic.type
    for topic in reader.get_all_topics_and_types()
}

message_classes = {
    topic: get_message(type_name)
    for topic, type_name in topic_types.items()
}

files = {}
writers = {}

try:
    while reader.has_next():
        topic, raw_data, timestamp_ns = reader.read_next()
        message = deserialize_message(raw_data, message_classes[topic])

        row = {
            "_bag_time_ns": timestamp_ns,
            "_bag_time_utc": datetime.fromtimestamp(
                timestamp_ns / 1_000_000_000,
                timezone.utc,
            ).isoformat(),
        }
        row.update(flatten(message_to_ordereddict(message)))

        if topic not in writers:
            filename = topic.strip("/").replace("/", "__") or "root"
            handle = open(
                output_dir / f"{filename}.csv",
                "w",
                newline="",
                encoding="utf-8",
            )
            files[topic] = handle
            writers[topic] = csv.DictWriter(handle, fieldnames=row.keys())
            writers[topic].writeheader()

        writers[topic].writerow(row)
finally:
    for handle in files.values():
        handle.close()

print(f"Exported {len(writers)} topics to {output_dir}")
