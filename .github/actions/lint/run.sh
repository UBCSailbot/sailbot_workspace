#!/bin/bash
set -e

source /opt/ros/${ROS2_DISTRO}/setup.bash
./setup.sh
if [[ "$LINTER" == "clang-tidy" ]]
then
    ./build.sh OFF
    ./run_clang-tidy.sh
else
    path=$(
        case "$LINTER" in
            ("lint_cmake") echo "src/network_systems/CMakeLists.txt src/network_systems/projects/ src/network_systems/lib/protofiles" ;;
            ("xmllint") echo "src/" ;;
            ("flake8") echo "src/py_pubsub_ex" ;;  # TODO replace paths with PATH and CTRL repos
            ("pep257") echo "src/py_pubsub_ex" ;;  # TODO replace paths with PATH and CTRL repos
        esac
    )
    ament_${LINTER} $path
fi
