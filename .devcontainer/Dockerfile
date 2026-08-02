# Gazebo backend selection.
#
# OPT-IN: the boat_simulator_gazebo physics backend (Gazebo Harmonic + asv_wave_sim) is a
# ~1.8 GB dependency closure that most of the team never touches, so it is not part of the
# base dev image. Enable it by uncommenting the INSTALL_GAZEBO build arg in
# docker-compose.yml, then rebuild the container.
# ---------------------------------------------------------------------------------------
ARG BASE_TAG=setup-included-2
ARG GAZEBO_TAG=gazebo
ARG INSTALL_GAZEBO=false

FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:${BASE_TAG} AS base-false
FROM ghcr.io/ubcsailbot/sailbot_workspace/gazebo-dev:${GAZEBO_TAG} AS base-true

FROM base-${INSTALL_GAZEBO}

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

# ---------------------------------------------------------------------------------------
# Gazebo Harmonic + asv_wave_sim, for the boat_simulator_gazebo physics backend.
#
# OPT-IN: gated on INSTALL_GAZEBO because the Gazebo dependency closure is ~1.8 GB and most
# of the team never runs the simulator. Enable it by exporting INSTALL_GAZEBO=true before
# the container build (docker-compose.yml forwards it as a build arg), then rebuild the
# container. When false, every step below is skipped and the image is unchanged.
#
# ROS_DISTRO is spelled out rather than inherited: it is exported by the ROS setup script
# that shells source at startup, which does not run during `docker build`, so relying on it
# here would silently expand to an empty string and ask apt for `ros--ros-gzharmonic`.
# ---------------------------------------------------------------------------------------
ARG INSTALL_GAZEBO=false
ARG GZ_DISTRO=harmonic
ARG ROS_DISTRO_NAME=humble
ARG ASV_WAVE_SIM_REF=master
ENV DEBIAN_FRONTEND=noninteractive

# Gazebo itself. The base image ships Ignition Fortress (ros-humble-ros-gz-*), which the
# Harmonic bridge Conflicts with; apt performs that swap here rather than at runtime.
RUN if [ "${INSTALL_GAZEBO}" = "true" ]; then \
        set -eux \
        && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
            -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
        && echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
            > /etc/apt/sources.list.d/gazebo-stable.list \
        && apt-get update \
        && apt-get install -y --no-install-recommends \
            gz-${GZ_DISTRO} \
            ros-${ROS_DISTRO_NAME}-ros-gz${GZ_DISTRO} \
            libcgal-dev \
            libfftw3-dev \
        && apt-get autoremove -y \
        && apt-get clean -y \
        && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*; \
    fi

# asv_wave_sim, prebuilt. It supplies the backend's flotation (the Hydrodynamics plugin)
# and its wave field, so it is required rather than optional once Gazebo is enabled.
#
# Built here rather than as a workspace source package because it is a pinned third-party
# dependency that never changes, and its CGAL-heavy translation units take ~15 minutes to
# compile. Paying that once in the image beats paying it on every fresh clone.
#
# Its system plugins have no CMake install rules (they are meant to be used straight from
# the build tree), so the built lib directory is copied out wholesale instead of running
# `cmake --install`. The models are plain data that CMake never installs either.
#
# The lib directory is registered with ldconfig because the plugins link against the
# libgz-waves1.so.1 core library sitting beside them. GZ_SIM_SYSTEM_PLUGIN_PATH only tells
# Gazebo where to find the plugins themselves; resolving what they in turn link against is
# the dynamic linker's job, and it does not search that directory otherwise. Without this,
# every plugin load fails with "libgz-waves1.so.1: cannot open shared object file".
RUN if [ "${INSTALL_GAZEBO}" = "true" ]; then \
        set -eux \
        && git clone --depth 1 --branch "${ASV_WAVE_SIM_REF}" \
            https://github.com/srmainwaring/asv_wave_sim.git /tmp/asv_wave_sim \
        && GZ_VERSION="${GZ_DISTRO}" cmake \
            -B /tmp/asv_wave_sim_build -S /tmp/asv_wave_sim/gz-waves \
            -DCMAKE_BUILD_TYPE=Release \
        && GZ_VERSION="${GZ_DISTRO}" cmake --build /tmp/asv_wave_sim_build \
            -- -j"$(nproc)" \
        && mkdir -p /opt/asv_wave_sim \
        && cp -a /tmp/asv_wave_sim_build/lib /opt/asv_wave_sim/lib \
        && cp -a /tmp/asv_wave_sim/gz-waves-models /opt/asv_wave_sim/gz-waves-models \
        && echo /opt/asv_wave_sim/lib > /etc/ld.so.conf.d/asv-wave-sim.conf \
        && ldconfig \
        && rm -rf /tmp/asv_wave_sim /tmp/asv_wave_sim_build; \
    fi

# Where Gazebo looks for asv_wave_sim's plugins and its `waves` model. Set unconditionally:
# when INSTALL_GAZEBO=false these paths simply do not exist, which Gazebo ignores.
#
# GZ_SIM_SYSTEM_PLUGIN_PATH covers the gz-sim systems (WavesModel, Hydrodynamics, ...),
# which run in the server and are looked up through gz-sim's own plugin loader.
#
# GZ_RENDERING_PLUGIN_PATH is separate and easy to miss: WavesVisual's ocean mesh is drawn
# by a render-engine *extension* (gz-waves1-rendering-ogre2) that only the GUI client
# process loads, through gz-rendering's RenderEngineExtensionManager rather than gz-sim's
# loader. That manager has no built-in search path of its own, so without this the GUI logs
# "Failed to load plugin [gz-waves1-rendering-ogre2] : couldn't load library on path []" and
# the boat model spawns but never renders (the scene is left in a broken state alongside the
# ocean surface it can't draw).
#
# GZ_VERSION is what asv_wave_sim reads to choose its Gazebo libraries; it is also set in
# devcontainer.json's containerEnv so an already-built image still exports it.
ENV GZ_VERSION=${GZ_DISTRO} \
    ASV_WAVE_SIM_PREFIX=/opt/asv_wave_sim \
    GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/asv_wave_sim/lib \
    GZ_RENDERING_PLUGIN_PATH=/opt/asv_wave_sim/lib \
    GZ_SIM_RESOURCE_PATH=/opt/asv_wave_sim/gz-waves-models/world_models:/opt/asv_wave_sim/gz-waves-models/models

# set user and working directory
USER ${USERNAME}
WORKDIR ${ROS_WORKSPACE}
