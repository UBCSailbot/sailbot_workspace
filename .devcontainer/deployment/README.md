# Deployment

## Deployment container commands

- Create a container using [`start_container.sh`](#start_containersh)
- Exit out of a container: `exit`
- Start an existing container: `docker start -ia <container ID>`
- Delete an existing container: `docker rm <container ID>`
- Find the container ID of a container: `docker ps -a`

## Deploy software

> These commands are run in the in the root directory of this repository

1. Run setup script: `./setup.sh`
2. Run quick build script: `./quick_build.sh`
3. Source ROS overlay: `source install/setup.bash`

## Development using deployment container

The deployment container isn't intended for development, but if you discover a bug and want to quickly push a fix:

1. Fix the issue in a terminal ***outside*** the deployment container
2. Run the software to verify your fix in a terminal ***inside*** the deployment container
3. Commit and push your fix in a terminal ***outside*** the deployment container

## Scripts

### `start_container.sh`

Run the [`base`](./../base-dev/base-dev.Dockerfile) image without VSCode for deployment:

- Runs the base image used by the Dev Container by default: `./start_container.sh`
- Run a specific version of the base image by specifying its ID: `./start_container.sh <IMAGE_ID>`
