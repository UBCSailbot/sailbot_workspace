FROM ghcr.io/ubcsailbot/sailbot_workspace/pre-base:ros_humble-ompl_4c86b2f as base

# install base apt dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        can-utils \
        clang \
        cmake \
        git \
        iproute2 \
        libboost-all-dev \
        libprotobuf-dev \
        protobuf-compiler \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/* \
    && rosdep init || echo "rosdep already initialized"
ENV DEBIAN_FRONTEND=

# root bash configuration
ENV ROS_WORKSPACE=/workspaces/sailbot_workspace
COPY update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc \
    && sync \
    && /bin/bash -c /sbin/update-bashrc \
    && rm /sbin/update-bashrc

FROM base as local-base

# install virtual iridium dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python2 \
        socat \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
RUN wget https://bootstrap.pypa.io/pip/2.7/get-pip.py \
    && python2 get-pip.py && pip2 install pyserial && pip2 install requests \
    && rm -f get-pip.py
ENV DEBIAN_FRONTEND=

# set virtual iridium environment variables
ENV LOCAL_TRANSCEIVER_TEST_PORT="/tmp/local_transceiver_test_port"
ENV VIRTUAL_IRIDIUM_PORT="/tmp/virtual_iridium_port"

FROM local-base as ros-dev

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
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc
    # Sourcing overlay in update-bashrc.sh instead
    # && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

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

# ros bash configuration
COPY update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc \
    && chown ros /sbin/update-bashrc \
    && sync \
    && /bin/bash -c /sbin/update-bashrc \
    && rm /sbin/update-bashrc

# install some clang tools and googletest for network systems
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        clang \
        clangd \
        clang-tidy \
        cmake \
        googletest \
        libboost-all-dev \
        libprotobuf-dev \
        protobuf-compiler \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

# install other helpful apt packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        less \
        openssh-client \
        tmux \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=
