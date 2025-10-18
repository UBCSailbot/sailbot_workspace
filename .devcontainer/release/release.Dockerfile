FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:latest

WORKDIR ${ROS_WORKSPACE}
COPY scripts/ ./scripts
ARG CACHEBUST
COPY src/ ./src

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && ./scripts/setup.sh \
    && ./scripts/build.sh"

USER ros
WORKDIR ${ROS_WORKSPACE}
