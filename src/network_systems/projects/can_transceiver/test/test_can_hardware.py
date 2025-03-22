import argparse
import random
import subprocess

# *************** CONSTANTS ***************
# Upper and lower bounds

# Bounds for Latitude and Longitude
LAT_LBND = -90.0
LAT_UBND = 90.0
LON_LBND = -180.0
LON_UBND = 180.0

# Bounds for Speed
SPEED_LBND = -10.0  # Placeholder number
SPEED_UBND = 10.0  # Placeholder number

# Bounds for Heading
HEADING_LBND = 0.0
HEADING_UBND = 360.0

# Boat rotation (See documentation on rate of turn)
ROT_LBND = -126
ROT_UBND = 126

# Boat dimension
SHIP_DIMENSION_LBND = 1.0  # arbitrary number
SHIP_DIMENSION_UBND = 650.0  # arbitrary number

# Bounds for Battery
BATT_VOLT_LBND = 0.5  # Placeholder number
BATT_VOLT_UBND = 250.0  # Placeholder number
BATT_CURR_LBND = -200.0  # Placeholder number
BATT_CURR_UBND = 200.0  # Placeholder number

# Bounds for Wind Sensor
WIND_DIRECTION_LBND = -180
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

    return None


def gen_data_wind_cmd():
    return None


def gen_rudder_data_frame_cmd():
    return None


def gen_sail_ais_cmd():
    return None


def gen_path_gps_data_frame_cmd():
    return None


def run_command(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    stdout, stderr = process.communicate()

    stdout = stdout.decode("utf-8")
    stderr = stderr.decode("utf-8")

    # Return the error code, stdout, and stderr
    return process.returncode, stdout


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
        "BMS_DATA_FRAME",
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

    t_info = {}
    test_setup(frames, t_info)


if __name__ == "__main__":
    main()
