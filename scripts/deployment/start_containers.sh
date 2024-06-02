#!/bin/bash

# TODO: check if it works without internet once built

## CLI ARGUMENTS ---

# default argument values
BASE_TAG="latest"
WEBSITE_ARG=""
INTERACTIVE=false
CLEAN=false
RESET=false

# function to display usage help
usage() {
    echo "Usage: $0 [--tag tag] [--website] [--interactive] [--clean] [--reset]"
    echo "  --tag          Specify the base Docker image tag (default: latest)"
    echo "  --website      If set, runs the website container"
    echo "  --interactive  If set, run commands inside the sailbot workspace container interactively"
    echo "  --clean        If set, removes containers after running"
    echo "  --reset        If set, removes containers and volumes after running"
    exit 1
}

# parse command-line options
while getopts ":-:" opt; do
    case "${opt}" in
        -)
            case "${OPTARG}" in
                tag)
                    BASE_TAG="${!OPTIND}"; OPTIND=$(( $OPTIND + 1 ))
                    ;;
                website)
                    WEBSITE_ARG="--file .devcontainer/website/docker-compose.website.yml"
                    ;;
                interactive)
                    INTERACTIVE=true
                    ;;
                clean)
                    CLEAN=true
                    ;;
                reset)
                    RESET=true
                    ;;
                *)
                    usage  # Show usage if argument is not recognized
                    ;;
            esac
            ;;
        *)
            usage  # Show usage if option is unknown
            ;;
    esac
done

# check for any remaining arguments after option parsing
if [ "$OPTIND" -le "$#" ]; then
    usage
fi

## HELPER VARIABLES ---

# get absolute path to workspace root in host OS
SCRIPT_DIR="$(dirname "$(readlink --canonicalize "$0")")"
HOST_WORKSPACE_ROOT="$SCRIPT_DIR/../.."

# common arguments for docker compose commmands
PROJECT_NAME="deployment"
DOCKER_COMPOSE_ARGS="docker compose --project-name $PROJECT_NAME --file .devcontainer/docker-compose.yml $WEBSITE_ARG"


## RUN COMMANDS ---

# run commands in the workspace's root directory
cd $HOST_WORKSPACE_ROOT

# start containers
SW_TAG=$BASE_TAG $DOCKER_COMPOSE_ARGS up --build --detach

# run commands inside sailbot workspace container
if [[ "$INTERACTIVE" = true ]]; then
    $DOCKER_COMPOSE_ARGS exec --interactive --tty sailbot-workspace /bin/bash
else
    $DOCKER_COMPOSE_ARGS exec --no-TTY sailbot-workspace /bin/bash -c "\
    source /opt/ros/\$ROS_DISTRO/setup.bash && \
    ./scripts/setup.sh && \
    ./scripts/build.sh && \
    source ./install/local_setup.bash && \
    ros2 launch src/global_launch/main_launch.py"
fi

# stop containers
$DOCKER_COMPOSE_ARGS stop

# remove containers
if [[ "$CLEAN" == "true" ]] || [[ "$RESET" == "true" ]]; then
    $DOCKER_COMPOSE_ARGS down
fi

# remove volumes
if [[ "$RESET" = true ]]; then
    docker volume ls -q | grep "^${PROJECT_NAME}_" | xargs -r docker volume rm
fi
