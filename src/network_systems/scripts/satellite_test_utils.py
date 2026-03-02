# Satellite Test Utilities
#
# Description:
#   In encode mode:
#     - Generates a protobuf GlobalPath object from:
#         • Provided waypoint coordinates, or
#         • Randomly generated waypoints
#
#   In decode mode:
#     - Decodes a protobuf byte string into a human-readable format
#
# Usage:
#   Encode or decode protobuf messages.
#
# Options:
#   -h, --help
#       Show this help message and exit
#
#   -m MODE, --mode MODE
#       Select mode: encode or decode
#
#   -w WAYPOINTS [WAYPOINTS ...], --waypoints WAYPOINTS [WAYPOINTS ...]
#       Waypoints as a flat list of latitude/longitude pairs
#       Example:
#           --waypoints 48.5 -123.4 49.1 -122.8
#
#   -b BYTES, --bytes BYTES
#       Hex-encoded protobuf bytes to decode
#
# Examples:
#   Decode:
#       python3 satellite_test_utils.py -m decode --bytes 120012070d6ff0a53f107e
#
#   Encode:
#       python3 satellite_test_utils.py -m encode


import argparse
import random
import subprocess

import custom_interfaces.msg as ci


def encode(waypoints=None):
    if waypoints:
        print(f"Encoding {len(waypoints)} provided waypoints...")
        path = ci.Path()
        wps = []
        for lat, lon in waypoints:
            waypoint = ci.HelperLatLon()
            waypoint.latitude, waypoint.longitude = lat, lon
            print(f"Latitude: {lat}    Longitude: {lon}")
            wps.append(waypoint)
        path.waypoints = wps
    else:
        print("Creating random global path...")
        n = random.randint(1, 3)
        print(f"Creating {n} waypoints...")
        path = ci.Path()
        wps = []
        for _ in range(n):
            lat = round(random.uniform(-180.0, 180.0), 2)
            lon = round(random.uniform(-90.0, 90.0), 2)
            waypoint = ci.HelperLatLon()
            waypoint.latitude, waypoint.longitude = lat, lon
            print(f"Latitude: {lat}    Longitude: {lon}")
            wps.append(waypoint)
        path.waypoints = wps

    proto_bytes = path_to_proto_bytes(path)
    print(f"\nEncoded protobuf bytes (hex):\n{proto_bytes.hex()}")


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


def decode(hex_bytes):
    import struct

    proto_bytes = bytes.fromhex(hex_bytes)

    proc = subprocess.Popen(
        ["protoc", "--decode_raw"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    stdout, stderr = proc.communicate(input=proto_bytes)

    if stderr:
        print(f"Error decoding protobuf: {stderr.decode()}")
        return

    print("Decoded protobuf message:")
    for line in stdout.decode().splitlines():
        # look for hex values like 0x3fa5f06f and convert to float
        if "0x" in line:
            hex_val = int(line.split("0x")[1].strip(), 16)
            float_val = struct.unpack("f", struct.pack("I", hex_val))[0]
            field = line.split(":")[0].strip()
            print(f"  {field}: {float_val}")
        else:
            print(f"  {line}")


def main():
    parser = argparse.ArgumentParser(description="Encode or decode protobuf messages")
    parser.add_argument("-m", "--mode", type=str, required=True, help="encode/decode")
    parser.add_argument(
        "-w",
        "--waypoints",
        type=float,
        nargs="+",
        help="Waypoints as flat list of lat lon pairs, e.g. --waypoints 48.5 -123.4 49.1 -122.8",
    )
    parser.add_argument(
        "-b",
        "--bytes",
        type=str,
        help="Hex-encoded protobuf bytes to decode",
    )

    args = parser.parse_args()

    if args.mode == "encode":
        waypoints = None
        if args.waypoints:
            if len(args.waypoints) % 2 != 0:
                print("Error: waypoints must be provided as lat lon pairs")
                return
            waypoints = list(zip(args.waypoints[::2], args.waypoints[1::2]))
        encode(waypoints)
    elif args.mode == "decode":
        if not args.bytes:
            print("Error: --bytes required for decode mode")
            return
        decode(args.bytes)
    else:
        print("Invalid mode argument: encode/decode only")


if __name__ == "__main__":
    main()
