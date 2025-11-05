FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:setup-included-2
ARG USERNAME=ros
ARG HOME=/home/${USERNAME}
WORKDIR ${ROS_WORKSPACE}

COPY scripts/ ./scripts

# CACHEBUST forces Docker to invalidate the cache for this layer.
# This ensures that changes in src/ are picked up during the build.
# CACHEBUST is defined as a build-arg set to the current timestamp.
ARG CACHEBUST
COPY src/ ./src

RUN chown -R ${USERNAME}:${USERNAME} ${ROS_WORKSPACE} ${HOME}
USER $USERNAME

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/build.sh"
WORKDIR ${ROS_WORKSPACE}
