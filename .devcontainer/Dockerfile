# Gazebo backend selection.
#
# OPT-IN: the boat_simulator_gazebo physics backend (Gazebo Harmonic + asv_wave_sim) is a
# ~1.8 GB dependency closure that most of the team never touches, so it is not part of the
# base dev image. Enable it by adding gazebo/docker-compose.gazebo.yml to
# "dockerComposeFile" in devcontainer.json, then rebuild the container.
# ---------------------------------------------------------------------------------------
ARG BASE_TAG=setup-included-2
ARG GAZEBO_TAG=gazebo
ARG INSTALL_GAZEBO=false

FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:${BASE_TAG} AS base-false
FROM ghcr.io/ubcsailbot/sailbot_workspace/gazebo-dev:${GAZEBO_TAG} AS base-true

FROM base-${INSTALL_GAZEBO}

# C++20 module toolchain.
USER root
RUN wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key \
        | gpg --dearmor -o /usr/share/keyrings/llvm-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/llvm-archive-keyring.gpg] http://apt.llvm.org/jammy/ llvm-toolchain-jammy-16 main" \
        > /etc/apt/sources.list.d/llvm-16.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        clang-16 \
        clangd-16 \
        clang-tidy-16 \
        clang-tools-16 \
    && pip3 install --no-cache-dir \
        cmake==3.28.4 \
        ninja==1.11.1.1 \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Copy configuration files (e.g., .vimrc) from config/ to the container's home directory
ARG USERNAME=ros
ARG HOME=/home/${USERNAME}
COPY --chown=${USERNAME}:${USERNAME} config ${HOME}

# # Uncomment this section to install additional apt packages
# ENV DEBIAN_FRONTEND=nonintesudo
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
