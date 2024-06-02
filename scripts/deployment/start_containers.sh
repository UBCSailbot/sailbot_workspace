#!/bin/bash

# TODO: check if it works without internet once built
# TODO: argument for deleting containers
# TODO: argument for deleting volumes

## CLI ARGUMENTS ---

# default argument values
BASE_TAG="latest"
WEBSITE_ARG=""
INTERACTIVE=""

# function to display usage help
usage() {
    echo "Usage: $0 [--tag tag] [--website]"
    echo "  --tag          Specify the base Docker image tag (default: latest)"
    echo "  --website      If set, runs the website container"
    echo "  --interactive  If set, run commands inside the sailbot workspace container interactively"
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
                    INTERACTIVE="interactive"
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


## HELPER VARIABLES ---

# get absolute path to workspace root in host OS
SCRIPT_DIR="$(dirname "$(readlink --canonicalize "$0")")"
HOST_WORKSPACE_ROOT="$SCRIPT_DIR/../.."

# common arguments for docker compose commmands
DOCKER_COMPOSE_ARGS="docker compose --project-name deployment --file .devcontainer/docker-compose.yml $WEBSITE_ARG"


## RUN COMMANDS ---

# run commands in the workspace's root directory
cd $HOST_WORKSPACE_ROOT

# start containers
SW_TAG=$BASE_TAG $DOCKER_COMPOSE_ARGS up --detach

# run commands inside sailbot workspace container
if [[ -n $INTERACTIVE ]]; then
    $DOCKER_COMPOSE_ARGS exec --interactive --tty sailbot-workspace /bin/bash
else
    $DOCKER_COMPOSE_ARGS exec -T sailbot-workspace /bin/bash -c "\
    cd \$ROS_WORKSPACE && \
    pwd && \
    ./scripts/setup.sh && \
    ./scripts/build.sh && \
    ./scripts/test.sh && \
    ls"
fi

# stop containers
$DOCKER_COMPOSE_ARGS stop
