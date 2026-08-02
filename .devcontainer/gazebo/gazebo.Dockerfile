# Gazebo Harmonic + asv_wave_sim, for the boat_simulator_gazebo physics backend. Published
# as its own image (ghcr.io/ubcsailbot/sailbot_workspace/gazebo-dev) so opting in to the
# simulator pulls a prebuilt layer instead of a ~15 minute local asv_wave_sim compile. See
# README.md to build/publish updates.
#
# ROS_DISTRO is spelled out rather than inherited: it's only exported by the setup script
# sourced at shell startup, which doesn't run during `docker build`, so it would otherwise
# silently expand empty and ask apt for `ros--ros-gzharmonic`.
ARG BASE_TAG=setup-included-2
FROM ghcr.io/ubcsailbot/sailbot_workspace/dev:${BASE_TAG}

ARG GZ_DISTRO=harmonic
ARG ROS_DISTRO_NAME=humble
ARG ASV_WAVE_SIM_REF=master
ENV DEBIAN_FRONTEND=noninteractive

# Gazebo itself. The base image ships Ignition Fortress (ros-humble-ros-gz-*), which
# Conflicts with the Harmonic bridge, so apt swaps it here rather than at runtime.
RUN set -eux \
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
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# asv_wave_sim, prebuilt: supplies the Hydrodynamics plugin and wave field that
# boat_simulator_gazebo needs. Built here (not as a workspace package) since it's a pinned
# third-party dependency whose CGAL-heavy build takes ~15 minutes - worth paying once in the
# image rather than on every clone or rebuild.
#
# Its plugins have no CMake install rules (used straight from the build tree), so the lib
# dir is copied out wholesale and registered with ldconfig - the plugins link against
# libgz-waves1.so.1 beside them, which GZ_SIM_SYSTEM_PLUGIN_PATH alone doesn't help the
# dynamic linker resolve. Without ldconfig, every plugin load fails with "libgz-waves1.so.1:
# cannot open shared object file".
RUN set -eux \
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
    && rm -rf /tmp/asv_wave_sim /tmp/asv_wave_sim_build

# Where Gazebo finds asv_wave_sim's plugins and its `waves` model.
#
# GZ_SIM_SYSTEM_PLUGIN_PATH covers the server-side gz-sim systems (WavesModel,
# Hydrodynamics). GZ_RENDERING_PLUGIN_PATH is easy to miss: the GUI client loads
# WavesVisual's render-engine extension through a separate loader with no search path of its
# own; without it, the GUI fails to load gz-waves1-rendering-ogre2 and the ocean never
# renders.
#
# GZ_VERSION is also set in devcontainer.json's containerEnv so an already-built image still
# exports it.
ENV GZ_VERSION=${GZ_DISTRO} \
    ASV_WAVE_SIM_PREFIX=/opt/asv_wave_sim \
    GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/asv_wave_sim/lib \
    GZ_RENDERING_PLUGIN_PATH=/opt/asv_wave_sim/lib \
    GZ_SIM_RESOURCE_PATH=/opt/asv_wave_sim/gz-waves-models/world_models:/opt/asv_wave_sim/gz-waves-models/models
