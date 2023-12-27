import argparse
import os
import signal
import subprocess
import sys
import time

import yaml

ROS_LAUNCH_CMD = "ros2 launch {} main_launch.py"
ROS_PACKAGES = ["boat_simulator", "controller", "local_pathfinding", "network_systems"]
ROS_WORKSPACE_PATH = os.getenv("ROS_WORKSPACE", default="/workspaces/sailbotworkspace")


def get_ros_launch_cmd(package_name, launch_config_files):
    launch_cmd = ROS_LAUNCH_CMD.format(package_name)

    if launch_config_files is not None:
        config_files_str = "".join(launch_config_files)
        launch_cmd += " --config:={}".format(config_files_str)

    return launch_cmd


def launch_modules(packages):
    launch_cmds = list()
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

    procs = list()
    for cmd in launch_cmds:
        procs.append(
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                shell=True,
                preexec_fn=os.setsid,
            )
        )  # Use os.setsid to set process group
    return procs


def stop_modules(process_list):
    for proc in process_list:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config", required=True, metavar="path", dest="config_files")
    args = parser.parse_args()

    for config_file in args.config_files:
        with open(config_file, "r") as config:
            params = yaml.safe_load(config)

            procs = launch_modules(params["required_packages"])

            time.sleep(10)
            stop_modules(procs)


if __name__ == "__main__":
    main()
