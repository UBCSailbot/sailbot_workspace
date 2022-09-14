# Base Image

## Features

- Starts from the [ros:eloquent-ros-base-bionic](https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/eloquent/ubuntu/bionic/ros-base/Dockerfile)
image
- Installs OMPL Python bindings on Python 3

## How to Build

```sh
docker build --tag ghcr.io/ubcsailbot/sailbot_workspace/base:ros_eloquent-ompl_1_5_0_python3 .devcontainer/base
```

Note: this container takes around 1 hour to build from scratch
