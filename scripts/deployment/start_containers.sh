#!/bin/bash

## CLI ARGUMENTS ---

# default argument values
WEBSITE_ARG=""
INTERACTIVE=false
CLEAN=false
RESET=false

# function to display usage help
usage() {
    echo "Usage: $0 [--website] [--interactive] [--clean] [--reset]"
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

## HELPER VARIABLES AND FUNCTIONS ---

# get absolute path to workspace root in host OS
SCRIPT_DIR="$(dirname "$(readlink --canonicalize "$0")")"
HOST_WORKSPACE_ROOT="$SCRIPT_DIR/../.."

# common arguments for docker compose commmands
MONGO_TAG="7"
PROJECT_NAME="deployment"
DOCKER_COMPOSE_ARGS="docker compose --project-name $PROJECT_NAME --file .devcontainer/docker-compose.yml $WEBSITE_ARG"

# function to pull the first FROM image from a specified Dockerfile
pull_from_image() {
    # Check if Dockerfile path was provided as an argument
    if [[ -z "$1" ]]; then
        echo "No Dockerfile path specified."
        return 1  # Return with error status
    fi

    # The path to the Dockerfile is the first argument to the function
    local dockerfile_path="$1"

    # Extract the image from the Dockerfile
    local image=$(grep '^FROM' "$dockerfile_path" | head -n 1 | awk '{print $2}')

    # Check if the image variable is not empty
    if [[ -n "$image" ]]; then
        echo "Pulling Docker image: $image"
        docker pull "$image"
    else
        echo "No image found in Dockerfile at $dockerfile_path."
        return 1  # Return with error status
    fi
}


## RUN COMMANDS ---

# run commands in the workspace's root directory
cd $HOST_WORKSPACE_ROOT

# pull images if connected to the internet
if wget -q --spider --timeout=1 http://google.com; then
    pull_from_image .devcontainer/Dockerfile
    pull_from_image .devcontainer/website/website.Dockerfile
    docker pull mongo:$MONGO_TAG
fi

# start containers
MONGO_TAG=$MONGO_TAG $DOCKER_COMPOSE_ARGS up --build --detach --pull never

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
