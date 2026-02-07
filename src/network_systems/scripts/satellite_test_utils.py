import argparse
import random
import subprocess

import custom_interfaces.msg as ci


def encode(filepath):
    print("Creating random global path...")

    n = random.randint(1, 3)
    print(f"Creating {n} waypoints...")
    path = ci.Path()
    waypoints = []

    for i in range(0, n):
        lat = round(random.uniform(-180.0, 180.0), 2)
        lon = round(random.uniform(-90.0, 90.0), 2)
        waypoint = ci.HelperLatLon()
        waypoint.latitude, waypoint.longitude = lat, lon
        print(f"Latidude: {lat}    Longitude: {lon}")

        waypoints.append(waypoint)

    path.waypoints = waypoints

    path_pb = path_to_proto_bytes(path)
    print(path_pb)

    with open(filepath, "wb") as f:
        f.write(path_pb)

    print(f"Protobuf serialized to {filepath}")


def path_to_proto_bytes(path):
    proc = subprocess.Popen(
        ["./build_path_proto"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
    )

    for wp in path.waypoints:
        proc.stdin.write(f"{wp.latitude} {wp.longitude}\n".encode())

    proc.stdin.close()
    proto_bytes = proc.stdout.read()

    return proto_bytes


# def decode(filepath):


def main():
    parser = argparse.ArgumentParser(description="Encode or decode protobuf messages")

    parser.add_argument("-m", "--mode", type=str, required=True, help="encode/decode")
    parser.add_argument(
        "-f", "--filepath", type=str, required=True, help="Input/Destination file path"
    )
    # parser.add_argument('--verbose', action='store_true', help="Enable verbose output")

    args = parser.parse_args()

    if args.mode == "encode":
        encode(args.filepath)
    elif args.mode == "decode":
        encode(args.filepath)
    else:
        print("Invalid mode argument: encode/decode only")


if __name__ == "__main__":
    main()
