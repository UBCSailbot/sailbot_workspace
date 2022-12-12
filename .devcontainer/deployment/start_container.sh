#!/bin/bash
# Run the base image without VSCode for deployment
# Runs the base image used by the Dev Container by default
# (Optional) run this script with an image ID as the first argument to run a specific version of the image

IMAGE_ID=${1:-""}

BASE_IMAGE="ghcr.io/ubcsailbot/sailbot_workspace/base"
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
HOST_WORKSPACE_ROOT="$SCRIPT_DIR/../.."
DOCKERFILE_DIR="$HOST_WORKSPACE_ROOT/.devcontainer"
DOCKERFILE_PATH="$DOCKERFILE_DIR/Dockerfile"
DEVCONTAINER_PATH="$DOCKERFILE_DIR/devcontainer.json"
CONTAINER_WORKSPACE_PATH="/workspaces/sailbot_workspace"
# run interactive terminal and name it sailbot
CONTAINER_NAME="sailbot"
DOCKER_RUN_CMD="docker run -it --name $CONTAINER_NAME"
# args to mount the repo inside the container
DOCKER_MNT_VOL_ARGS="-v $HOST_WORKSPACE_ROOT:$CONTAINER_WORKSPACE_PATH -w $CONTAINER_WORKSPACE_PATH"
DOCKER_RUN_CMD="$DOCKER_RUN_CMD $DOCKER_MNT_VOL_ARGS"

# Stop any currently running or exited container that was started by this script
docker stop $CONTAINER_NAME > /dev/null
docker rm $CONTAINER_NAME > /dev/null

# Function to fork off and setup CAN devices
# I do not recommend printing to the terminal inside this function as it will print onto the interactive command line
setup_can_fork()
{
    # Takes a CAN device ID as the first input argument and returns the command to set it up within the docker container
    setup_can_cmd()
    {
        DEV_ID=$1
        if [[ $DEV_ID == "vcan0" ]]
        then
            TYPE_ARG="vcan"
        elif [[ $DEVID == "can0" ]]
        then
            TYPE_ARG="can bitrate 12500" # TODO: No clue what the bitrate should be as physical CAN is not setup yet
        fi
        echo "docker exec -d $CONTAINER_NAME ip link add dev $DEV_ID type $TYPE_ARG && ip link set up $DEV_ID"
    }

    if [[ $(ifconfig | grep -w vcan0) != "" ]]
    then
        $(setup_can_cmd vcan0)
    fi
    if [[ $(ifconfig | grep -w can0) != "" ]]
    then
        $(setup_can_cmd can0)
    fi
}

# Parse the devcontainer.json file for docker run arguments
get_docker_run_args()
{
    DOCKER_RUN_ARGS=$(sed -n '/"runArgs": \[/,/\],/p' $DEVCONTAINER_PATH) # isolate the entire runArgs field
    DOCKER_RUN_ARGS=$(sed 's/"runArgs": \[//g' <<< "$DOCKER_RUN_ARGS")    # remove "runArgs": [
    DOCKER_RUN_ARGS=$(sed 's/\],//g' <<< "$DOCKER_RUN_ARGS")              # remove ],
    DOCKER_RUN_ARGS=$(sed 's/"//g' <<< "$DOCKER_RUN_ARGS")                # remove "
    DOCKER_RUN_ARGS=$(sed 's/,//g' <<< "$DOCKER_RUN_ARGS")                # remove ,

    echo $DOCKER_RUN_ARGS # return args
}
# Parse the Dockerfile to get the base/dev image tag
get_dev_tag()
{
    TAG=$(head -n1 $DOCKERFILE_PATH) # Get first line with the image source and tag
    IFS=':' read -ra TAG <<< "$TAG"  # Turn the string into an array
    TAG=${TAG[1]}                    # Get the tag from the second element in the array

    echo $TAG # return tag
}

DOCKER_RUN_ARGS=$(get_docker_run_args)
DOCKER_RUN_CMD="$DOCKER_RUN_CMD $DOCKER_RUN_ARGS"

if [[ $IMAGE_ID == "" ]]
then
    TAG=$(get_dev_tag)
    # Run latest base image
    DOCKER_RUN_CMD="$DOCKER_RUN_CMD $BASE_IMAGE:$TAG /bin/bash"
else
    DOCKER_RUN_CMD="$DOCKER_RUN_CMD $IMAGE_ID /bin/bash"
fi

# Pretty hacky. Setup CAN devices by forking off from this script, waiting one second for $DOCKER_RUN_CMD to start
# the container, and executing the commands to setup the device(s)
(sleep 1 && setup_can_fork) &

echo "Running: $DOCKER_RUN_CMD"
$DOCKER_RUN_CMD # Any commands placed after this will run AFTER the container is exited
