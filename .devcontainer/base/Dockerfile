# build command: docker build --tag ghcr.io/UBCSailbot/vscode_ros2_workspace/base:ros_eloquent-ompl_1_5_0_python3 .
FROM ros:eloquent-ros-base-bionic

# install OMPL
COPY ./install-ompl-ubuntu-python3.sh /ompl/
WORKDIR /ompl
RUN ./install-ompl-ubuntu-python3.sh --python

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
WORKDIR /
