#!/bin/bash
# Build and run the sailbot_workspace image without VSCode for deployment
# (Optional) run this script with an image ID as the first argument to run a specific version of the image

IMAGE_ID=${1:-""}

IMAGE_NAME="sailbot_workspace"
HOST_WORKSPACE_ROOT=$(readlink -f ../) # store as absolute path
DOCKERFILE_DIR="$HOST_WORKSPACE_ROOT/.devcontainer"
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

DOCKER_RUN_ARGS=$(get_docker_run_args)

if [[ $IMAGE_ID == "" ]]
then
    # Build IMAGE_NAME from dockerfile found in DOCKERFILE_DIR (does nothing if image is already built)
    docker build -t $IMAGE_NAME $DOCKERFILE_DIR
    # Run latest IMAGE_NAME in an interactive bash terminal 
    echo "Running: docker run -it $DOCKER_MNT_VOL_ARGS $DOCKER_RUN_ARGS $IMAGE_NAME /bin/bash"
    docker run -it $DOCKER_MNT_VOL_ARGS $DOCKER_RUN_ARGS $IMAGE_NAME /bin/bash
else
    echo "Running: docker run -it $DOCKER_MNT_VOL_ARGS $DOCKER_RUN_ARGS $IMAGE_ID /bin/bash"
    docker run -it $DOCKER_MNT_VOL_ARGS $DOCKER_RUN_ARGS $IMAGE_ID /bin/bash
fi
