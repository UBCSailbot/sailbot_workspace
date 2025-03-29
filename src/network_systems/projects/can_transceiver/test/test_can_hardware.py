import argparse
import random
import re
import subprocess
import time

# *************** CONSTANTS ***************

LOG_FILE = "test_logs.txt"
# Upper and lower bounds

# Bounds for Latitude and Longitude
LAT_LBND = 0  # -90.0
LAT_UBND = 90.0
LON_LBND = 0  # -180.0
LON_UBND = 180.0

# Bounds for Speed
SPEED_LBND = 0  # -10.0  # Placeholder number
SPEED_UBND = 10.0  # Placeholder number

# Bounds for Heading
HEADING_LBND = 0.0
HEADING_UBND = 360.0

# Boat rotation (See documentation on rate of turn)
ROT_LBND = 0  # -126
ROT_UBND = 126

# Boat dimension
SHIP_DIMENSION_LBND = 1.0  # arbitrary number
SHIP_DIMENSION_UBND = 650.0  # arbitrary number

# Bounds for Battery
BATT_VOLT_LBND = 0.5  # Placeholder number
BATT_VOLT_UBND = 250.0  # Placeholder number
BATT_CURR_LBND = 0  # -200.0  # Placeholder number
BATT_CURR_UBND = 200.0  # Placeholder number

# Bounds for Wind Sensor
WIND_DIRECTION_LBND = 0  # -180
WIND_DIRECTION_UBND = 179


CMDS = {
    "BMS_DATA_FRAME": "cansend can1 030##8{}",
    "SAIL_WIND": "cansend can1 040##8{}",
    "DATA_WIND": "cansend can1 041##8{}",
    "RUDDER_DATA_FRAME": "cansend can1 050##8{}",
    "SAIL_AIS": "cansend can1 060##8{}",
    "PATH_GPS_DATA_FRAME": "cansend can1 070##8{}",
}


def gen_bms_data_frame_cmd():
    cmd_dict = {}
    for i in range(0, 3):
        voltage = round(random.uniform(BATT_VOLT_LBND, BATT_VOLT_UBND), 2) * 100
        current = round(random.uniform(BATT_CURR_LBND, BATT_CURR_UBND), 2) * 100
        voltage_byte = int(voltage).to_bytes(4, byteorder="little")
        current_byte = int(current).to_bytes(4, byteorder="little")
        cmd_dict[i] = {
            "voltage": voltage / 100,
            "current": current / 100,
            "bytes": (voltage_byte + current_byte).hex().upper(),
        }
    return cmd_dict


def gen_sail_wind_cmd():
    WIND_SPEED_UBND, WIND_SPEED_LBND = 200, 0  # kts
    cmd_dict = {}
    for i in range(0, 3):
        wind_angle = random.randint(WIND_DIRECTION_LBND, WIND_DIRECTION_UBND)
        wind_speed = round(random.uniform(WIND_SPEED_LBND, WIND_SPEED_UBND), 2) / 10
        angle_byte = int(wind_angle).to_bytes(2, byteorder="little")
        speed_byte = int(wind_speed).to_bytes(2, byteorder="little")
        cmd_dict[i] = {
            "wind_angle": wind_angle,
            "wind_speed": wind_speed * 10,
            "bytes": (angle_byte + speed_byte).hex().upper(),
        }
    return cmd_dict


def gen_data_wind_cmd():
    WIND_SPEED_UBND, WIND_SPEED_LBND = 200, 0  # kts
    cmd_dict = {}
    for i in range(0, 3):
        wind_angle = random.randint(WIND_DIRECTION_LBND, WIND_DIRECTION_UBND)
        wind_speed = round(random.uniform(WIND_SPEED_LBND, WIND_SPEED_UBND), 2) / 10
        angle_byte = int(wind_angle).to_bytes(2, byteorder="little")
        speed_byte = int(wind_speed).to_bytes(2, byteorder="little")
        cmd_dict[i] = {
            "wind_angle": wind_angle,
            "wind_speed": wind_speed * 10,
            "bytes": (angle_byte + speed_byte).hex().upper(),
        }
    return cmd_dict


def gen_rudder_data_frame_cmd():
    cmd_dict = {}
    for i in range(0, 3):
        true_heading = round(random.uniform(HEADING_LBND, HEADING_UBND), 2) * 1000
        heading_byte = int(true_heading).to_bytes(4, byteorder="little")
        cmd_dict[i] = {
            "true_heding": true_heading / 1000,
            "bytes": (heading_byte).hex().upper(),
        }
    return cmd_dict


def gen_sail_ais_cmd():
    cmd_dict = {}
    for i in range(0, 3):
        ship_id = random.randint(0, 10000)
        lat = (random.randint(LAT_LBND, LAT_UBND) + 90) * 1000000
        lon = (random.randint(LON_LBND, LON_UBND) + 180) * 1000000
        sog = round(random.uniform(SPEED_LBND, SPEED_UBND), 2) * 10
        cog = random.randint(HEADING_LBND, HEADING_UBND) * 10
        heading = random.randint(HEADING_LBND, HEADING_UBND)
        rot = random.randint(ROT_LBND, ROT_UBND)
        length = random.randint(SHIP_DIMENSION_LBND, SHIP_DIMENSION_UBND)
        width = random.randint(SHIP_DIMENSION_LBND, SHIP_DIMENSION_UBND)
        total_ships = random.randint(1, 127)
        ship_idx = random.randint(0, total_ships)

        id_byte = int(ship_id).to_bytes(4, byteorder="little")
        lat_byte = int(lat).to_bytes(4, byteorder="little")
        lon_byte = int(lon).to_bytes(4, byteorder="little")
        sog_byte = int(sog).to_bytes(2, byteorder="little")
        cog_byte = int(cog).to_bytes(2, byteorder="little")
        heading_byte = int(heading).to_bytes(4, byteorder="little")
        rot_byte = int(rot).to_bytes(1, byteorder="little")
        length_byte = int(length).to_bytes(2, byteorder="little")
        width_byte = int(width).to_bytes(2, byteorder="little")
        total_ships_byte = int(total_ships).to_bytes(1, byteorder="little")

        cmd_dict[i] = {
            "ship_id": ship_id,
            "lat": lat,
            "lon": lon,
            "sog": sog,
            "cog": cog,
            "heading": heading,
            "rot": rot,
            "length": length,
            "width": width,
            "total_ships": total_ships,
            "ship_idx": ship_idx,
            "bytes": (
                id_byte
                + lat_byte
                + lon_byte
                + sog_byte
                + cog_byte
                + heading_byte
                + rot_byte
                + length_byte
                + width_byte
                + total_ships_byte
            )
            .hex()
            .upper(),
        }
    return cmd_dict


def gen_path_gps_data_frame_cmd():
    cmd_dict = {}
    for i in range(0, 3):
        lat = (random.randint(LAT_LBND, LAT_UBND) + 90) * 1000000
        lon = (random.randint(LON_LBND, LON_UBND) + 180) * 1000000
        seconds = round(random.uniform(0, 60), 2) * 1000
        minutes = random.randint(0, 60)
        hours = random.randint(0, 24)
        sog = round(random.uniform(SPEED_LBND, SPEED_UBND), 2) * 1000

        lat_byte = int(lat).to_bytes(4, byteorder="little")
        lon_byte = int(lon).to_bytes(4, byteorder="little")
        seconds_byte = int(seconds).to_bytes(4, byteorder="little")
        minutes_byte = int(minutes).to_bytes(1, byteorder="little")
        hours_byte = int(hours).to_bytes(1, byteorder="little")
        reserved_byte = int(0).to_bytes(2, byteorder="little")
        sog_byte = int(sog).to_bytes(4, byteorder="little")
        cmd_dict[i] = {
            "lat": lat,
            "lon": lon,
            "seconds": seconds,
            "minutes": minutes,
            "hours": hours,
            "sog": sog,
            "bytes": (
                lat_byte
                + lon_byte
                + seconds_byte
                + minutes_byte
                + hours_byte
                + reserved_byte
                + sog_byte
            )
            .hex()
            .upper(),
        }
    return cmd_dict


def run_command(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    stdout, stderr = process.communicate()

    stdout = stdout.decode("utf-8")
    stderr = stderr.decode("utf-8")

    return process.returncode, stdout


def capture_log(process):
    # Continuously capture the logs while the process is running
    with open(LOG_FILE, "w") as f:
        while True:
            # Read the output from the process' stdout and stderr
            output = process.stdout.readline()
            error = process.stderr.readline()
            if output:
                f.write(output)  # Write stdout to the log file
            if error:
                f.write(error)  # Write stderr to the log file
            # If the process ends (manually or otherwise), break out of the loop
            if output == "" and error == "" and process.poll() is not None:
                break
            time.sleep(0.1)  # Avoid tight loop, allows other processes to run


def setup_can():
    r_code, output = run_command(
        "$ROS_WORKSPACE/sailbot_workspace/scripts/deployment/helpers/setup_can.sh"
    )
    return r_code, output


def test_setup(frames, t_info):
    for frame in frames:
        if frame == "BMS_DATA_FRAME":
            t_info[frame] = gen_bms_data_frame_cmd()
        elif frame == "SAIL_WIND":
            t_info[frame] = gen_sail_wind_cmd()
        elif frame == "DATA_WIND":
            t_info[frame] = gen_data_wind_cmd()
        elif frame == "RUDDER_DATA_FRAME":
            t_info[frame] = gen_rudder_data_frame_cmd()
        elif frame == "SAIL_AIS":
            t_info[frame] = gen_sail_ais_cmd()
        elif frame == "PATH_GPS_DATA_FRAME":
            t_info[frame] = gen_path_gps_data_frame_cmd()


def result_verify(frames, t_info, process):

    with open(LOG_FILE, "r") as log_file:
        log_data = log_file.readlines()

    res_data = {}
    patterns = {
        "BMS_DATA_FRAME": re.compile(r"\[BATTERY\] Voltage: ([\d.]+)"),
        "SAIL_WIND": re.compile(r"\[WIND SENSOR\] Speed: ([\d.]+) Angle: ([\d.]+)"),
        "PATH_GPS_DATA_FRAME": re.compile(
            r"\[GPS\] Latitude: ([\d.]+) Longitude: ([\d.]+) Speed: ([\d.]+)"
        ),
        "SAIL_AIS": re.compile(
            r"\[AIS SHIP\] ID: ([\d.]+) Latitude: ([\d.]+) Longitude: ([\d.]+)"
        ),
        "RUDDER_DATA_FRAME": re.compile(r"\[DESIRED HEADING\] Heading: ([\d.]+)"),
    }
    filtered_patterns = {frame: patterns[frame] for frame in frames if frame in patterns}

    for line in log_data:
        for frame, pattern in filtered_patterns.items():
            match = pattern.search(line)
            if match:
                if frame == "BMS_DATA_FRAME":
                    res_data[frame].append({"voltage": float(match.group(1))})
                elif frame == "MAIN_TRIM_TAB":
                    res_data[frame].append({"angle": float(match.group(1))})
                elif frame == "SAIL_WIND":
                    res_data[frame].append(
                        {
                            "wind_speed": float(match.group(1)),
                            "wind_angle": float(match.group(2)),
                        }
                    )
                elif frame == "PATH_GPS_DATA_FRAME":
                    res_data[frame].append(
                        {
                            "lat": float(match.group(1)),
                            "lon": float(match.group(2)),
                            "speed": float(match.group(3)),
                        }
                    )
                elif frame == "SAIL_AIS":
                    res_data[frame].append(
                        {
                            "ship_id": match.group(1),
                            "lat": float(match.group(2)),
                            "lon": float(match.group(3)),
                        }
                    )
                elif frame == "RUDDER_DATA_FRAME":
                    res_data[frame].append({"heading": float(match.group(1))})

    for frame, info in t_info.items():
        if frame == "BMS_DATA_FRAME":
            for i in range(0, 3):
                if res_data[frame][i]["voltage"] != info[i]["voltage"]:
                    print(
                        f"Voltage mismatch for {frame}: {res_data[frame][i]['voltage']} \
                            != {info[i]['voltage']}"
                    )
                    return -1
        elif frame == "MAIN_TRIM_TAB":
            for i in range(0, 3):
                if res_data[frame][i]["angle"] != info[i]["angle"]:
                    print(
                        f"Angle mismatch for {frame}: {res_data[frame][i]['angle']} \
                            != {info[i]['angle']}"
                    )
                    return -1
        elif frame == "SAIL_WIND":
            for i in range(0, 3):
                if res_data[frame][i]["wind_speed"] != info[i]["wind_speed"]:
                    print(
                        f"Wind speed mismatch for {frame}: {res_data[frame][i]['wind_speed']} \
                            != {info[i]['wind_speed']}"
                    )
                    return -1
                if res_data[frame][i]["wind_angle"] != info[i]["wind_angle"]:
                    print(
                        f"Wind angle mismatch for {frame}: {res_data[frame][i]['wind_angle']} \
                            != {info[i]['wind_angle']}"
                    )
                    return -1
        elif frame == "PATH_GPS_DATA_FRAME":
            for i in range(0, 3):
                if res_data[frame][i]["lat"] != info[i]["lat"]:
                    print(
                        f"Latitude mismatch for {frame}: {res_data[frame][i]['lat']} \
                            != {info[i]['lat']}"
                    )
                    return -1
                if res_data[frame][i]["lon"] != info[i]["lon"]:
                    print(
                        f"Longitude mismatch for {frame}: {res_data[frame][i]['lon']} \
                            != {info[i]['lon']}"
                    )
                    return -1
                if res_data[frame][i]["speed"] != info[i]["sog"]:
                    print(
                        f"Speed mismatch for {frame}: {res_data[frame][i]['speed']} \
                            != {info[i]['sog']}"
                    )
                    return -1
        elif frame == "SAIL_AIS":
            for i in range(0, 3):
                if res_data[frame][i]["ship_id"] != info[i]["ship_id"]:
                    print(
                        f"Ship ID mismatch for {frame}: {res_data[frame][i]['ship_id']} \
                            != {info[i]['ship_id']}"
                    )
                    return -1
                if res_data[frame][i]["lat"] != info[i]["lat"]:
                    print(
                        f"Latitude mismatch for {frame}: {res_data[frame][i]['lat']} \
                            != {info[i]['lat']}"
                    )
                    return -1
                if res_data[frame][i]["lon"] != info[i]["lon"]:
                    print(
                        f"Longitude mismatch for {frame}: {res_data[frame][i]['lon']} \
                            != {info[i]['lon']}"
                    )
                    return -1
        elif frame == "RUDDER_DATA_FRAME":
            for i in range(0, 3):
                if res_data[frame][i]["heading"] != info[i]["true_heading"]:
                    print(
                        f"Heading mismatch for {frame}: {res_data[frame][i]['heading']} \
                            != {info[i]['true_heading']}"
                    )
                    return -1

    return 0


def main():

    parser = argparse.ArgumentParser(description="Script to setup CAN or other actions.")

    parser.add_argument("-setup", action="store_true", help="Setup CAN")
    parser.add_argument("-frames", type=str, help="Comma-separated list of frames to specify")

    # Parse arguments
    args = parser.parse_args()

    if args.setup:
        r_code, output = setup_can()
        if r_code != 0:
            print(f"Error setting up CAN: {output}")
        else:
            print("CAN setup complete")
    else:
        print("CAN already set up")

    # List of all available frames
    frames = [
        # "BMS_DATA_FRAME",
        "SAIL_WIND",
        "DATA_WIND",
        "RUDDER_DATA_FRAME",
        "SAIL_AIS",
        "PATH_GPS_DATA_FRAME",
    ]

    # If -frames argument is provided, filter frames
    if args.frames:
        specified_frames = args.frames.split(",")
        frames = [frame for frame in frames if frame in specified_frames]
    print(frames)

    print("Starting ROS...")

    process = subprocess.Popen(
        ["ros2", "launch", "network_systems", "main_launch.py", "mode:=production"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    t_info = {}
    test_setup(frames, t_info)
    for frame, info in t_info.items():
        for i in range(0, 3):
            cmd = CMDS[frame].format(info[i]["bytes"])
            print(info[i])
            print(cmd)
            r_code, output = run_command(cmd)
            if r_code != 0:
                print(f"Error sending frame {frame}: {output}")
            else:
                print(f"Frame {frame} sent successfully")
    capture_log(process)
    if result_verify(frames, t_info, process) != 0:
        print("Test failed")
    else:
        print("Test passed")
    process.terminate()
    process.stdout.close()
    process.wait()


if __name__ == "__main__":
    main()
