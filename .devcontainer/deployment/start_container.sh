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
# args to mount the repo inside the container
DOCKER_MNT_VOL_ARGS="-v $HOST_WORKSPACE_ROOT:$CONTAINER_WORKSPACE_PATH -w $CONTAINER_WORKSPACE_PATH"

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

if [[ $IMAGE_ID == "" ]]
then
    TAG=$(get_dev_tag)
    # Run latest base image in an interactive bash terminal 
    echo "Running: docker run -it $DOCKER_MNT_VOL_ARGS $DOCKER_RUN_ARGS $BASE_IMAGE:$TAG /bin/bash"
    docker run -it $DOCKER_MNT_VOL_ARGS $DOCKER_RUN_ARGS $BASE_IMAGE:$TAG /bin/bash
else
    echo "Running: docker run -it $DOCKER_MNT_VOL_ARGS $DOCKER_RUN_ARGS $IMAGE_ID /bin/bash"
    docker run -it $DOCKER_MNT_VOL_ARGS $DOCKER_RUN_ARGS $IMAGE_ID /bin/bash
fi
