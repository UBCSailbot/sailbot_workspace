FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:ros_humble-ompl_d8375d8-amd64

# # Uncomment this section to install the packages used in the ROS 2 tutorials
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#  && apt-get -y install --no-install-recommends \
#  # Your package list here
#  ros-humble-turtlesim \
#  # Clean up
#  && apt-get autoremove -y \
#  && apt-get clean -y \
#  && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=

# # Uncomment this section to install additional packages
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#  && apt-get -y install --no-install-recommends \
#  # Your package list here
#  # Clean up
#  && apt-get autoremove -y \
#  && apt-get clean -y \
#  && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=

# Copy configuration files (e.g., .vimrc) from config/ to the container's home directory
ARG USERNAME=ros
ARG HOME=/home/$USERNAME
COPY config ${HOME}
