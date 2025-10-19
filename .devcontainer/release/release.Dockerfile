FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:latest

WORKDIR ${ROS_WORKSPACE}
COPY scripts/ ./scripts

# CACHEBUST forces Docker to invalidate the cache for this layer.
# This ensures that changes in src/ are picked up during the build.
# CACHEBUST is defined as a build-arg set to the current timestamp.
ARG CACHEBUST
COPY src/ ./src

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && ./scripts/setup.sh \
    && ./scripts/build.sh"

USER ros
WORKDIR ${ROS_WORKSPACE}
