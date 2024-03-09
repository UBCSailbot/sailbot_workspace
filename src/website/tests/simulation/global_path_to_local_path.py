"""Converts the global path to a series of local path waypoints."""

import argparse
import json


def read_json_file(input_file):
    with open(input_file, "r") as file:
        data = json.load(file)
    return data


def write_json_file(output_file, data):
    new_data = []

    for i in range(len(data[0]['waypoints'])):
        if i == len(data[0]['waypoints']) - 1:
            break
        new_data.append(
            {
                "waypoints": [
                    {
                        "latitude": data[0]['waypoints'][i]['latitude'],
                        "longitude": data[0]['waypoints'][i]['longitude']
                    },
                    {
                        "latitude": data[0]['waypoints'][i + 1]['latitude'],
                        "longitude": data[0]['waypoints'][i + 1]['longitude']
                    },
                ]
            }
        )
    with open(output_file, "w") as file:
        json.dump(new_data, file, indent=4)


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
