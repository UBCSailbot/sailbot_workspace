# UBC Sailbot Boat Simulator

[![Tests](https://github.com/UBCSailbot/boat_simulator/actions/workflows/tests.yml/badge.svg)](https://github.com/UBCSailbot/boat_simulator/actions/workflows/tests.yml)

UBC Sailbot's boat simulator for the new project. This repository contains a ROS package `boat_simulator`. This README
contains only setup and run instructions. Further information on the boat simulator can be found on the software
team's [docs website](https://ubcsailbot.github.io/docs/main/current/boat_simulator/overview/).

## Setup

The boat simulator is meant to be ran inside the [Sailbot Workspace](https://github.com/UBCSailbot/sailbot_workspace)
development environment. Follow the setup instructions for the Sailbot Workspace
[here](https://ubcsailbot.github.io/docs/main/current/sailbot_workspace/setup/)
to get started and build all the necessary ROS packages.

## Run

The [`launch/`](./launch/) folder contains a [ROS 2 launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
responsible for starting up the boat simulator. To run the boat simulator standalone, execute the launch file after building
the `boat_simulator` package:

``` shell
ros2 launch boat_simulator main_launch.py [OPTIONS]...
```

To see a list of options for simulator configuration, add the `-s` flag at the end of the above command.

## Test

Run the `test` task in the Sailbot Workspace. See [here](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette)
on how to run vscode tasks.
