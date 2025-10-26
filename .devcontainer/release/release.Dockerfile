FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:latest AS builder
WORKDIR ${ROS_WORKSPACE}
COPY scripts/ ./scripts
# CACHEBUST forces Docker to invalidate the cache for this layer.
# This ensures that changes in src/ are picked up during the build.
# CACHEBUST is defined as a build-arg set to the current timestamp.
ARG CACHEBUST
COPY src/ ./src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/build.sh"

FROM ubuntu:jammy-20240111 AS env
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
WORKDIR ${ROS_WORKSPACE}

FROM env AS import-build-artifacts
COPY --from=builder ${ROS_WORKSPACE}/build/ ./build
COPY --from=builder ${ROS_WORKSPACE}/install/ ./install
COPY --from=builder ${ROS_WORKSPACE}/log/ ./log
COPY --from=builder ${ROS_WORKSPACE}/src/ ./src
COPY --from=builder ${ROS_WORKSPACE}/scripts/ ./scripts
COPY --from=builder /opt/ros/humble /opt/ros/humble

# Install all runtime dependencies in a single layer
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        locales \
        tzdata \
        sudo \
        python3-numpy \
        can-utils \
        iproute2 \
        tmux \
        screen \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-pip \
        curl \
        gnupg2 \
        lsb-release \
        sudo \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        python3-argcomplete \
        python3-rosdep \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rosdep init || echo "rosdep already initialized"
ENV DEBIAN_FRONTEND=

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/setup.sh exec" \
    # downgrade setuptools
    # https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/?answer=400052#post-id-400052
    && pip3 install setuptools==58.2.0

# root bash configuration
COPY .devcontainer/base-dev/update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc \
    && sync \
    && /bin/bash -c /sbin/update-bashrc \
    && rm /sbin/update-bashrc

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Cleanup
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

ARG HOME=/home/$USERNAME
# persist ROS logs
RUN mkdir -p ${HOME}/.ros/log \
    && chown -R ${USERNAME} ${HOME}/.ros
USER ${USERNAME}
WORKDIR ${ROS_WORKSPACE}
