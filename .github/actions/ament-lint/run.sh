#!/bin/bash
set -e

# Display error message
function error {
    message=$1
    echo -e "\e[1;31m${message}\e[0m"
}

# Display warning message
function warn {
    message=$1
    echo -e "\e[1;33m${message}\e[0m"
}

function get_search_command {
    case ${LINTER} in
        lint_cmake) CMD='find ${VALID_SRC_DIRS_ARG} -type f \( -name "CMakeLists.txt" -o -name "*.cmake" -o -name "*.cmake.in" \)' ;;
        flake8) CMD='find ${VALID_SRC_DIRS_ARG} -type f -name "*.py"' ;;
        pep257) CMD='find ${VALID_SRC_DIRS_ARG} -type f -name "*.py"' ;;
        xmllint) CMD='find ${VALID_SRC_DIRS_ARG} -type f -name "*.xml"' ;;
        *) error "ERROR: Invalid linter ${LINTER} specified in ament-lint action"; exit 1 ;;
    esac

    echo $CMD
}

function lint {
    VALID_SRC_DIRS_ARG=$1
    FILE_SEARCH_CMD=`get_search_command`
    LINTED_FILES=`eval ${FILE_SEARCH_CMD}`

    if [[ ! -z ${LINTED_FILES} ]]; then
        ament_${LINTER} ${LINTED_FILES}
    else
        warn "WARNING: No files found for ${LINTER}. Skipping ament_${LINTER}..."
    fi
}

source /opt/ros/${ROS_DISTRO}/setup.bash
./setup.sh
cd src

# Exclude virtual_iridium because it is a legacy library and docs
VALID_SRC_DIRS=$(ls | grep -v -e virtual_iridium -e docs -e new_project.repos)
lint ${VALID_SRC_DIRS}
