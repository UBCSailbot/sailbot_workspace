FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:fix_dockerfile_name

# Uncomment this section to install the apt packages used in the ROS 2 tutorials
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y \
        # Your package list here
        ros-humble-turtlesim \
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

# Uncomment this section to install additional apt packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y \
        # Your package list here
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
ENV DEBIAN_FRONTEND=

# Copy configuration files (e.g., .vimrc) from config/ to the container's home directory
ARG USERNAME=ros
ARG HOME=/home/$USERNAME
COPY config ${HOME}
