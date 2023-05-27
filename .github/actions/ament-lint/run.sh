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

function do_lint_cmake {
    VALID_SRC_DIRS_ARG=$1
    LINTED_FILES=`find ${VALID_SRC_DIRS_ARG} -type f \( -name "CMakeLists.txt" -o -name "*.cmake" -o -name "*.cmake.in" \)`

    if [[ ! -z ${LINTED_FILES} ]]; then
        ament_lint_cmake $LINTED_FILES
    else
        warn "WARNING: No files found for lint_cmake. Skipping lint_cmake..."
    fi
}

function do_flake8 {
    VALID_SRC_DIRS_ARG=$1
    LINTED_FILES=`find ${VALID_SRC_DIRS_ARG} -type f -name "*.py"`

    if [[ ! -z ${LINTED_FILES} ]]; then
        ament_flake8 $LINTED_FILES
    else
        warn "WARNING: No files found for flake8. Skipping flake8..."
    fi
}

function do_pep257 {
    VALID_SRC_DIRS_ARG=$1
    LINTED_FILES=`find ${VALID_SRC_DIRS_ARG} -type f -name "*.py"`

    if [[ ! -z ${LINTED_FILES} ]]; then
        ament_pep257 $LINTED_FILES
    else
        warn "WARNING: No files found for pep257. Skipping pep257..."
    fi
}

function do_xmllint {
    VALID_SRC_DIRS_ARG=$1
    LINTED_FILES=`find ${VALID_SRC_DIRS_ARG} -type f -name "*.xml"`

    if [[ ! -z ${LINTED_FILES} ]]; then
        ament_xmllint $LINTED_FILES
    else
        warn "WARNING: No files found for xmllint. Skipping xmllint..."
    fi
}

source /opt/ros/${ROS_DISTRO}/setup.bash
./setup.sh
cd src

# exclude virtual_iridium because it is a legacy library
VALID_SRC_DIRS=$(ls | grep -v -e virtual_iridium -e new_project.repos)
case ${LINTER} in
    lint_cmake) do_lint_cmake $VALID_SRC_DIRS ;;
    flake8) do_flake8 $VALID_SRC_DIRS ;;
    pep257) do_pep257 $VALID_SRC_DIRS ;;
    xmllint) do_xmllint $VALID_SRC_DIRS ;;
    *) error "ERROR: Invalid linter ${LINTER} specified in ament-lint action"; exit 1 ;;
esac
