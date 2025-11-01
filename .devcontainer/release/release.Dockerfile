FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:setup-included-2

WORKDIR ${ROS_WORKSPACE}

COPY --chown=ros:ros scripts/ ./scripts

# CACHEBUST forces Docker to invalidate the cache for this layer.
# This ensures that changes in src/ are picked up during the build.
# CACHEBUST is defined as a build-arg set to the current timestamp.
ARG CACHEBUST
COPY --chown=ros:ros src/ ./src

USER ros

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/build.sh"
WORKDIR ${ROS_WORKSPACE}
