FROM ghcr.io/ubcsailbot/sailbot_workspace/base:ros_humble-ompl_1bb0aa2-amd64 as ros-dev

# From https://github.com/athackst/dockerfiles/blob/32a872348af0ad25ec4a6e6184cb803357acb6ab/ros2/humble.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    pylint \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    vim \
    wget \
    # Install ros distro testing packages
    ros-humble-ament-lint \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-launch-testing-ros \
    python3-autopep8 \
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
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

FROM ros-dev as dev

ARG USERNAME=ros
ARG HOME=/home/$USERNAME

# persist ROS logs
RUN mkdir -p ${HOME}/.ros/log \
    && chown -R ${USERNAME} ${HOME}/.ros

# persist bash history
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=${HOME}/commandhistory/.bash_history" \
    && mkdir ${HOME}/commandhistory \
    && touch ${HOME}/commandhistory/.bash_history \
    && chown ${USERNAME} ${HOME}/commandhistory/.bash_history \
    && echo $SNIPPET >> "${HOME}/.bashrc"

ARG ROS_WORKSPACE=/workspaces/sailbot_workspace

# bash configuration
COPY update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc ; chown ros /sbin/update-bashrc ; sync ; /bin/bash -c /sbin/update-bashrc ; rm /sbin/update-bashrc

# install clang, some clang tools, and protobuf for network systems
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y \
        clang \
        clang-tidy \
        clangd \
        googletest \
        libprotobuf-dev \
        protobuf-compiler \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

# install git-delta
RUN wget https://github.com/dandavison/delta/releases/download/0.14.0/git-delta-musl_0.14.0_amd64.deb
RUN dpkg -i git-delta-musl_0.14.0_amd64.deb
RUN rm git-delta-musl_0.14.0_amd64.deb

# install other helpful apt packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y \
        less \
        openssh-client \
        tmux \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=
