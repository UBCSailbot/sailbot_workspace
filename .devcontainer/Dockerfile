FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:develop

## Uncomment this section to install the packages used in the ROS 2 tutorials
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#     && apt-get -y install --no-install-recommends \
#     # Your package list here
#     ros-humble-turtlesim \
#     # Clean up
#     && apt-get autoremove -y \
#     && apt-get clean -y \
#     && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=

# Copy configuration files (e.g., .vimrc) from config/ to the container's home directory
ARG USERNAME=ros
ARG HOME=/home/$USERNAME
COPY config ${HOME}
