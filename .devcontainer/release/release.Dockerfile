FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:setup-included-2 AS builder
ARG USERNAME=ros
ARG HOME=/home/${USERNAME}
WORKDIR ${ROS_WORKSPACE}
COPY scripts/ ./scripts
# CACHEBUST forces Docker to invalidate the cache for this layer.
# This ensures that changes in src/ are picked up during the build.
# CACHEBUST is defined as a build-arg set to the current timestamp.
# ARG CACHEBUST
COPY src/ ./src
RUN chown -R ${USERNAME}:${USERNAME} ${ROS_WORKSPACE} ${HOME}
USER ${USERNAME}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/build.sh"

FROM ubuntu:jammy-20240111 AS env
ARG USERNAME=ros
ARG HOME=/home/${USERNAME}
ENV ROS_WORKSPACE=/workspaces/sailbot_workspace \
LANG=en_US.UTF-8 \
    TZ="America/Vancouver" \
    ROS_DISTRO=humble \
    AMENT_PREFIX_PATH=/opt/ros/humble \
    COLCON_PREFIX_PATH=/opt/ros/humble \
    LD_LIBRARY_PATH=/opt/ros/humble/lib \
    PATH=/opt/ros/humble/bin:$PATH \
    PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2 \
    AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 \
    LOCAL_TRANSCEIVER_TEST_PORT="/tmp/local_transceiver_test_port" \
    VIRTUAL_IRIDIUM_PORT="/tmp/virtual_iridium_port"

FROM env AS runtime-dependencies

ENV DEBIAN_FRONTEND=noninteractive

# Fix certificates
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Install language and timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update && apt-get install -y --no-install-recommends \
        locales \
        tzdata \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# Copy over ros2 instalation
# COPY --from=builder /opt/ros/humble /opt/ros/humble
# This unfortunately doesn't work out of the box.
# I think it can replace installing ros-humble-ros-base and python3-argcomplete.
# Which saves ~3 minutes.
# However, it causes a complete failure in launching the ros2 packages for some reason.

# Install ros2
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg2 \
        lsb-release \
        sudo \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Install all runtime dependencies in a single layer
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-dev \
        python3-numpy \
        python3-pip \
        tmux \
        screen \
        git \
        bash-completion \
        python3-colcon-common-extensions \
        python3-rosdep \
        iproute2 \
        can-utils \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/* \
    && rosdep init || echo "rosdep already initialized" \
    && pip3 install ompl==1.7.0 # pathfinding
ENV DEBIAN_FRONTEND=

FROM runtime-dependencies AS import-build-artifacts
ARG USERNAME=ros
ARG HOME=/home/${USERNAME}
# Create the non-root user: ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Cleanup
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

WORKDIR ${ROS_WORKSPACE}
COPY --from=builder ${ROS_WORKSPACE}/build/ ./build
COPY --from=builder ${ROS_WORKSPACE}/install/ ./install
COPY --from=builder ${ROS_WORKSPACE}/log/ ./log
COPY --from=builder ${ROS_WORKSPACE}/src/ ./src
COPY --from=builder ${ROS_WORKSPACE}/scripts/ ./scripts
RUN chown -R ${USERNAME}:${USERNAME} ${ROS_WORKSPACE} ${HOME}

FROM import-build-artifacts AS setup
# We need to make this directory because setup.sh can only create the file.
USER ${USERNAME}
RUN sudo mkdir -p /etc/ros/rosdep/sources.list.d \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/setup.sh exec" \
    # downgrade setuptools
    # https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/?answer=400052#post-id-400052
    && pip3 install setuptools==58.2.0
USER root

# root bash configuration
COPY .devcontainer/base-dev/update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc \
    && sync \
    && /bin/bash -c /sbin/update-bashrc \
    && rm /sbin/update-bashrc

ARG HOME=/home/$USERNAME
# persist ROS logs
RUN mkdir -p ${HOME}/.ros/log \
    && chown -R ${USERNAME} ${HOME}/.ros

USER ${USERNAME}
