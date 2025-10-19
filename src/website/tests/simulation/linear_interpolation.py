"""Interpolates points on a line between two waypoints. Used to create a series of GPS points
based on the local path."""

import argparse
import json
from math import radians, sin, cos, sqrt, atan2

from shapely.geometry import LineString, Point
from geopy.distance import geodesic


def get_points_along_line(start, end):
    line = LineString([start, end])

    points = [start]
    points.append((line.centroid.x, line.centroid.y))

    points.append(end)
    return points


def read_json_file(input_file):
    with open(input_file, "r") as file:
        data = json.load(file)
    return data


def write_json_file(output_file, data):
    doc = []
    for i in range(len(data)):
        start = data[i]["waypoints"][0]
        end = data[i]["waypoints"][1]
        waypoints = get_points_along_line(
            (start['latitude'], start['longitude']),
            (end['latitude'], end['longitude'])
        )
        waypoints.pop()

        start["speed"] = 0
        start["heading"] = 0
        end["speed"] = 0
        end["heading"] = 0

        for point in waypoints:
            doc.append({"latitude": point[0], "longitude": point[1], "speed": 0, "heading": 0})

    with open(output_file, "w") as file:
        json.dump(doc, file, indent=4)


def main():
    parser = argparse.ArgumentParser(
        description="Read from an input JSON file and write to an output JSON file"
    )
    parser.add_argument("input_file", help="Input JSON file name")
    parser.add_argument("output_file", help="Output JSON file name")
    args = parser.parse_args()

    input_file_name = args.input_file
    output_file_name = args.output_file

    # Read data from the input JSON file
    data_from_input = read_json_file(input_file_name)

    # Write data to the output JSON file
    write_json_file(output_file_name, data_from_input)

    print(f"Data has been read from {input_file_name} and written to {output_file_name}.")


if __name__ == "__main__":
    main()
