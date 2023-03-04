# Deployment

## Scripts

### `start_container.sh`

Runs the [`base`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/base-dev/base-dev.Dockerfile)
image using the run arguments in [`devcontainer.json`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/devcontainer.json).

If any CAN devices `can0` or `vcan0` are setup on the host computer, then this script will enable them inside the
container.

A new container is created every time this is run. The default container name is `sailbot`. Container names are unique,
so if you want to use multiple deployment containers (e.g., from different branches) you will have to update the variable
`CONTAINER_NAME` in the script.

Usage:

- Runs the base image used by the Dev Container by default: `./start_container.sh`
- Run a specific version of the base image by specifying its ID: `./start_container.sh <IMAGE_ID>`

## Deployment container commands

- Exit out of a container: `exit`
- Start an existing container: `docker start -ia <container name>`
- Delete an existing container: `docker rm <container name>`
- Find the container ID of a container: `docker ps -a`

## Deploy software

> These commands are run in the in the root directory of this repository

1. Run setup script: `./setup.sh`
2. Run quick build script: `./quick_build.sh`
3. Source ROS overlay: `source install/setup.bash`

## Develop software

The deployment container isn't intended for development, but if you discover a bug and want to quickly push a fix:

1. Fix the issue in a terminal ***outside*** the deployment container
2. Run the software to verify your fix in a terminal ***inside*** the deployment container
3. Commit and push your fix in a terminal ***outside*** the deployment container
