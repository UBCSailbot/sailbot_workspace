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
# args to support can/vcan
DOCKER_CAN_ARGS="--cap-add=NET_ADMIN --network=host"
DOCKER_RUN_CMD="$DOCKER_RUN_CMD $DOCKER_MNT_VOL_ARGS $DOCKER_CAN_ARGS"

# Parse the Dockerfile to get the base/dev image tag
get_dev_tag()
{
    TAG=$(head -n1 $DOCKERFILE_PATH) # Get first line with the image source and tag
    IFS=':' read -ra TAG <<< "$TAG"  # Turn the string into an array
    TAG=${TAG[1]}                    # Get the tag from the second element in the array

    echo $TAG # return tag
}

if [[ $(docker ps -a --filter "name=$CONTAINER_NAME" | grep -w $CONTAINER_NAME) != "" ]]
then
    echo "Error: $CONTAINER_NAME is already running or suspended. Run the following command and then try again:"
    echo "docker stop $CONTAINER_NAME; docker rm $CONTAINER_NAME"
    exit 1
fi

if [[ $IMAGE_ID == "" ]]
then
    TAG=$(get_dev_tag)
    DOCKER_RUN_CMD="$DOCKER_RUN_CMD --pull always $BASE_IMAGE:$TAG /bin/bash"
else
    DOCKER_RUN_CMD="$DOCKER_RUN_CMD $IMAGE_ID /bin/bash"
fi

echo "Running: $DOCKER_RUN_CMD"
$DOCKER_RUN_CMD # Any commands placed after this will run AFTER the container is exited
