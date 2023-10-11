#!/bin/bash
set -e

function error {
    # Display error message
    message=$1
    echo -e "\e[1;31m${message}\e[0m"
}

function warn {
    # Display warning message
    message=$1
    echo -e "\e[1;33m${message}\e[0m"
}

function get_search_command {
    # Returns a search command in the form of a string that performs a search for
    # files to lint. The returned search command depends on the specified linter.

    case ${LINTER} in
        lint_cmake) CMD='find ${VALID_SRC_DIRS_ARG} -type f \( -name "CMakeLists.txt" -o -name "*.cmake" -o -name "*.cmake.in" \)' ;;
        flake8) CMD='find ${VALID_SRC_DIRS_ARG} -type f -name "*.py"' ;;
        mypy) CMD='find ${VALID_SRC_DIRS_ARG} -type f -name "*.py"' ;;
        xmllint) CMD='find ${VALID_SRC_DIRS_ARG} -type f -name "*.xml"' ;;
        *) error "ERROR: Invalid linter ${LINTER} specified in ament-lint action"; exit 1 ;;
    esac

    echo $CMD
}

function lint {
    # Run a specified linter on a set of files
    # Arg 1: A list of source directories and files to perform linting on

    VALID_SRC_DIRS_ARG=$1
    FILE_SEARCH_CMD=`get_search_command`
    LINTED_FILES=`eval ${FILE_SEARCH_CMD}`

    if [[ ! -z ${LINTED_FILES} ]]; then
        ament_${LINTER} ${LINTED_FILES}
    else
        warn "WARNING: No files found for ${LINTER}. Skipping ament_${LINTER}..."
    fi
}

if [[ $LOCAL_RUN != "true" ]]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    ./setup.sh
fi

# Exclude repos and files we don't want to lint
VALID_SRC_DIRS=$(ls src | grep -v -e virtual_iridium -e docs -e raye-local-pathfinding -e website -e notebooks -e polaris.repos)

# Loop over each directory and lint it
for dir in $VALID_SRC_DIRS; do
    echo "Run $LINTER on src/$dir"
    lint src/$dir || warn "WARNING: $LINTER errors in src/$dir, continuing with others"
done
