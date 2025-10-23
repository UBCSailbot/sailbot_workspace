FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:latest

# Copy configuration files (e.g., .vimrc) from config/ to the container's home directory
ARG USERNAME=ros
ARG HOME=/home/${USERNAME}
COPY --chown=${USERNAME}:${USERNAME} config ${HOME}

# # Uncomment this section to install additional apt packages
# ENV DEBIAN_FRONTEND=nonintesudo dpkg --configure -aractive
# RUN apt-get update \
#     && apt-get install -y --no-install-recommends \
#         # Your package list here
#         # Uncomment the line below to install the apt package used in the ROS 2 tutorials
#         # ros-humble-turtlesim \
#     # Clean up
#     && apt-get autoremove -y \
#     && apt-get clean -y \
#     && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# set user and working directory
USER ${USERNAME}
WORKDIR ${ROS_WORKSPACE}
