# build command: docker build --tag ghcr.io/UBCSailbot/vscode_ros2_workspace/base:raye .
FROM ghcr.io/UBCSailbot/vscode_ros2_workspace/base:ros_eloquent-ompl_1_5_0_python3 as builder

# install ROS 1
# https://github.com/osrf/docker_images/blob/master/ros/eloquent/ubuntu/bionic/ros1-bridge/Dockerfile

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV ROS1_DISTRO melodic
ENV ROS2_DISTRO eloquent

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-comm \
    ros-melodic-roscpp-tutorials \
    ros-melodic-rospy-tutorials \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-eloquent-ros1-bridge \
    ros-eloquent-demo-nodes-cpp \
    ros-eloquent-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

# install downstream packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-rosdep \
    && rm -rf /var/lib/apt/lists/*

FROM builder

# install Raye local pathfinding dependencies
# https://github.com/UBCSailbot/local-pathfinding/tree/master/install
COPY ./install-ompl-ubuntu-python2.sh /ompl-raye/
COPY ./install-raye-dependencies.bash /ompl-raye/
WORKDIR /ompl-raye
RUN ./install-raye-dependencies.bash

# clean image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
WORKDIR /
