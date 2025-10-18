FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:latest

WORKDIR ${ROS_WORKSPACE}
COPY src/ ./src
COPY scripts/ ./scripts

# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/setup.sh"
# RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/build.sh"

# RUN rm -rf ./scripts \
#     && rm -rf ./src

# ARG USERNAME=ros
# ARG HOME=./home/${USERNAME}
# COPY --chown=${USERNAME}:${USERNAME} config ${HOME}

# USER ${USERNAME}
# WORKDIR ${ROS_WORKSPACE}
