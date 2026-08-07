#!/bin/bash
#!/usr/bin/env bash
set -e

# Usage:
#   ./install_deps.sh exec   # installs only exec_depend deps
#   ./install_deps.sh        # installs all deps (default)

# Choose dependency mode based on argument
if [ "$1" = "exec" ]; then
    DEP_FLAGS="--dependency-types exec -y"
    echo "Installing only runtime (exec_depend) dependencies..."
else
    DEP_FLAGS="-y"
    echo "Installing all dependency types (build, exec, test, etc.)..."
fi

# Create/overwrite the custom rosdep list file
CUSTOM_ROSDEP_LIST="/etc/ros/rosdep/sources.list.d/20-sailbot.list"
CUSTOM_ROSDEP_FILE="custom-rosdep.yaml"
echo "# sailbot" | sudo tee "$CUSTOM_ROSDEP_LIST" > /dev/null

for DIR in "$ROS_WORKSPACE"/src/*; do
    if [ -d "$DIR" ]; then
        FILE="$DIR/$CUSTOM_ROSDEP_FILE"
        if [ -f "$FILE" ]; then
            echo "Adding $FILE to $CUSTOM_ROSDEP_LIST"
            echo "yaml file://$(realpath "$FILE")" | sudo tee --append "$CUSTOM_ROSDEP_LIST" > /dev/null
        fi
    fi
done

# Opt-in Gazebo physics backend: import source dependencies (e.g. asv_wave_sim) declared in
# per-package dependencies.repos files. Opt-in because they require Gazebo (gz-sim)
# development libraries; see src/boat_simulator_gazebo/README.md.
if [ "${ENABLE_GAZEBO:-false}" = "true" ]; then
    if ! command -v vcs > /dev/null; then
        echo "ENABLE_GAZEBO=true requires vcstool (pip install vcstool)" >&2
        exit 1
    fi
    # The Dev Container image prebuilds asv_wave_sim into ASV_WAVE_SIM_PREFIX when it is
    # built with INSTALL_GAZEBO=true. When that is present there is nothing to import or
    # compile, so skip the source checkout entirely; importing it would only add a
    # 15-minute CGAL build that produces the same plugins the image already ships.
    ASV_WAVE_SIM_PREFIX="${ASV_WAVE_SIM_PREFIX:-/opt/asv_wave_sim}"
    if [ -d "$ASV_WAVE_SIM_PREFIX" ]; then
        echo "asv_wave_sim provided by the image at $ASV_WAVE_SIM_PREFIX: skipping source import"
    else
        for REPOS_FILE in "$ROS_WORKSPACE"/src/*/dependencies.repos; do
            if [ -f "$REPOS_FILE" ]; then
                echo "Importing repositories from $REPOS_FILE"
                vcs import "$ROS_WORKSPACE/src" < "$REPOS_FILE"
            fi
        done
    fi

    # Guard an in-workspace asv_wave_sim checkout, which only exists on images built
    # without INSTALL_GAZEBO or for someone hacking on the wave physics locally.
    #
    # It needs Gazebo's development libraries (gz-cmake3 and friends; both Harmonic and
    # Garden ship gz-cmake3). Without them its gz-waves package fails to configure, and
    # because colcon aborts dependent packages on failure that would break the build of
    # the entire workspace. Mark it ignored unless those libraries are present, so a
    # missing dependency degrades to "no wave field" instead of "nothing builds".
    ASV_WAVE_SIM_DIR="$ROS_WORKSPACE/src/asv_wave_sim"
    if [ -d "$ASV_WAVE_SIM_DIR" ]; then
        # `cmake --find-package` writes a CMakeFiles/ directory into the working
        # directory, so probe from a scratch directory rather than the caller's.
        GZ_PROBE_DIR="$(mktemp -d)"
        if (cd "$GZ_PROBE_DIR" && cmake --find-package -DNAME=gz-cmake3 \
            -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST) > /dev/null 2>&1; then
            echo "Found gz-cmake3: building asv_wave_sim"
            rm -f "$ASV_WAVE_SIM_DIR/COLCON_IGNORE"
            # asv_wave_sim picks its Gazebo library versions from GZ_VERSION and falls back
            # to Garden's (gz-math7 and friends) when it is unset, which does not configure
            # against Harmonic. Warn rather than fail: the variable is needed at build time,
            # not here.
            if [ "${GZ_VERSION:-}" != "harmonic" ]; then
                echo "WARNING: GZ_VERSION is '${GZ_VERSION:-unset}', expected 'harmonic'." \
                    "asv_wave_sim will fail to configure without it." \
                    "Set it in .devcontainer/devcontainer.json or export it before building."
            fi
        else
            echo "gz-cmake3 not found: skipping asv_wave_sim (no wave field)." \
                "Install the Gazebo Harmonic development libraries to enable it."
            touch "$ASV_WAVE_SIM_DIR/COLCON_IGNORE"
        fi
        rm -rf "$GZ_PROBE_DIR"
    fi
fi

sudo apt-get update
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" $DEP_FLAGS

# Create logging folder for all ROS logs.
LOG_PATH="$ROS_WORKSPACE/src/global_launch/voyage_log"
if [ ! -d "$LOG_PATH" ]; then
    sudo mkdir -p "$LOG_PATH"
fi

