FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:setup-included AS builder
WORKDIR ${ROS_WORKSPACE}
COPY scripts/ ./scripts
# CACHEBUST forces Docker to invalidate the cache for this layer.
# This ensures that changes in src/ are picked up during the build.
# CACHEBUST is defined as a build-arg set to the current timestamp.
ARG CACHEBUST
COPY src/ ./src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/build.sh"

FROM ubuntu:jammy-20240111 AS runtime
COPY --from=builder build/ ./build
COPY --from=builder install/ ./install
COPY --from=builder log/ ./log
COPY --from=builder /opt/ros/humble /opt/ros/humble

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    TZ="America/Vancouver" \
    ROS_WORKSPACE=/workspaces/sailbot_workspace \
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

# Install all runtime dependencies in a single layer
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        locales \
        tzdata \
        python3-numpy \
        can-utils \
        iproute2 \
        tmux \
        screen \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

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
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    # Cleanup
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

ARG HOME=/home/$USERNAME
# persist ROS logs
RUN mkdir -p ${HOME}/.ros/log \
    && chown -R ${USERNAME} ${HOME}/.ros
USER ${USERNAME}
WORKDIR ${ROS_WORKSPACE}
