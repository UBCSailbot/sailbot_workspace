# build using Build Dev Image workflow
FROM ghcr.io/ubcsailbot/vscode_ros2_workspace/base:raye as builder

# develop configuration from athackst's image
# https://github.com/athackst/dockerfiles/blob/main/ros2/eloquent.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  pylint3 \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  vim \
  wget \
  # Install ros distro testing packages
  ros-eloquent-ament-lint \
  ros-eloquent-launch-testing \
  ros-eloquent-launch-testing-ament-cmake \
  ros-eloquent-launch-testing-ros \
  python-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized" \
  # Update pydocstyle
  && pip install --upgrade pydocstyle

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc
  # && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

FROM builder

ENV USER ros
USER ros
ENV HOME /home/${USER}

# persist ROS logs
RUN mkdir -p ${HOME}/.ros/log \
    && chown -R ${USER} ${HOME}/.ros/log

# persist bash history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=${HOME}/commandhistory/.bash_history" \
  && mkdir ${HOME}/commandhistory \
  && touch ${HOME}/commandhistory/.bash_history \
  && chown -R ${USER} ${HOME}/commandhistory \
  && echo $SNIPPET >> "/home/${USER}/.bashrc"

ARG ROS1_WORKSPACE=/workspaces/catkin_ws
ARG ROS2_WORKSPACE=/workspaces/vscode_ros2_workspace

# bash configuration
COPY update-bashrc.sh /sbin/update-bashrc
RUN sudo chmod +x /sbin/update-bashrc ; sudo chown ros /sbin/update-bashrc ; sync ; /bin/bash -c /sbin/update-bashrc ; sudo rm /sbin/update-bashrc

# ROS 1 workspace setup
WORKDIR $ROS1_WORKSPACE
RUN mkdir src
RUN git clone https://github.com/UBCSailbot/local-pathfinding.git src/local-pathfinding
RUN git clone https://github.com/UBCSailbot/boat_controller.git src/boat_controller
RUN git clone https://github.com/UBCSailbot/sailbot-msg.git src/sailbot-msg
RUN /bin/bash -c "source /opt/ros/${ROS1_DISTRO}/setup.bash; catkin_make"

# ROS 2 workspace setup
WORKDIR $ROS2_WORKSPACE
