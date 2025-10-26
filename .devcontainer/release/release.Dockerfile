FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:latest AS builder
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

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Install language
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y --no-install-recommends tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-numpy \
        can-utils \
        iproute2 \
        tzdata \
        tmux \
        screen \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

# root bash configuration
ENV ROS_WORKSPACE=/workspaces/sailbot_workspace
COPY .devcontainer/base-dev/update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc \
    && sync \
    && /bin/bash -c /sbin/update-bashrc \
    && rm /sbin/update-bashrc

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

# downgrade setuptools
# https://answers.ros.org/question/396439/setuptoolsdeprecationwarning-setuppy-install-is-deprecated-use-build-and-pip-and-other-standards-based-tools/?answer=400052#post-id-400052
RUN pip3 install setuptools==58.2.0

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
COPY .devcontainer/base-dev/update-bashrc.sh /sbin/update-bashrc
RUN chmod +x /sbin/update-bashrc \
    && chown ros /sbin/update-bashrc \
    && sync \
    && /bin/bash -c /sbin/update-bashrc \
    && rm /sbin/update-bashrc

ENV DEBIAN_FRONTEND=
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV TZ="America/Vancouver"
