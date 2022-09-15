# Base Image for Raye Migration

## Features

- Starts from the [base:ros_eloquent-ompl_1_5_0_python3](../) image
- Installs ROS 1 Melodic and the ros1-bridge for ROS 2 Eloquent
- Installs Raye's local pathfinding dependencies, including OMPL Python bindings on Python 2

## How to Build

```sh
docker build --tag ghcr.io/ubcsailbot/sailbot_workspace/base:raye .devcontainer/base/raye
```

Note: this container takes around 10 hours to build from scratch
