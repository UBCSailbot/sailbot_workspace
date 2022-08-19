# FROM althack/ros2:foxy-dev
FROM ghcr.io/ubcsailbot/vscode_ros2_workspace/dev:latest

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN sudo apt-get update \
#  && sudo apt-get -y install --no-install-recommends \
#  # Your package list here
#  # Clean up
#  && sudo apt-get autoremove -y \
#  && sudo apt-get clean -y \
#  && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=

# Copy configuration files (e.g., .vimrc) from config/ to the container's home directory
COPY config ${HOME}
