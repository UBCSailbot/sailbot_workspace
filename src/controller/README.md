# Controller

UBC Sailbot's controller for the new project. This repository contains a ROS package `controller`. This README
contains only setup and run instructions. Further information on the controller can be found on the software
team's [docs website](https://ubcsailbot.github.io/sailbot_workspace/main/current/controller/overview/).

## Setup

The controller is meant to be ran inside the [Sailbot Workspace](https://github.com/UBCSailbot/sailbot_workspace)
development environment. Follow the setup instructions for the Sailbot Workspace
[here](https://ubcsailbot.github.io/sailbot_workspace/main/current/sailbot_workspace/usage/setup/)
to get started and build all the necessary ROS packages.

## Run

The [`launch/`](./launch/) folder contains a [ROS 2 launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
responsible for starting up the controller. To run the controller standalone, execute the launch file after building
the `controller` package:

``` shell
ros2 launch controller main_launch.py [OPTIONS]...
```

To see a list of options for configuration, add the `-s` flag at the end of the above command.

## Test

Run the `test` task in the Sailbot Workspace. See [here](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette)
on how to run vscode tasks.
