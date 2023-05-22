# Deployment

## Scripts

### `start_container.sh`

Runs the [`base`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/base-dev/base-dev.Dockerfile)
image. A new container is created every time this is run. The default container name is `sailbot`. Container names are unique,
so if you want to use multiple deployment containers (e.g., from different branches) you will have to update the variable
`CONTAINER_NAME` in the script.

Usage:

- Runs the base image used by the Dev Container by default: `./start_container.sh`
- Run a specific version of the base image by specifying its ID: `./start_container.sh <IMAGE_ID>`

### `setup_boot.sh`

Configures programs and scripts that need to run when the main computer boots. Only needs to be run once unless the
script is updated. Does not need to be rerun if any scripts or programs it targets are updated, with the exception of
renaming or moving the file.

Usage:

- Must be run as root
- `sudo ./setup_boot.sh`

## Deployment container commands

- Exit out of a container: `exit`
- Start an existing container: `docker start -ia <container name>`
- Delete an existing container: `docker rm <container name>`
- Find the container ID of a container: `docker ps -a`

## Deploy software

> These commands are run in the in the root directory of this repository

1. Run the setup script: `./setup.sh`
2. Run the quick build script: `./quick_build.sh`

## Develop software

The deployment container isn't intended for development, but if you discover a bug and want to quickly push a fix:

1. Fix the issue in a terminal ***outside*** the deployment container
2. Run the software to verify your fix in a terminal ***inside*** the deployment container
3. Commit and push your fix in a terminal ***outside*** the deployment container
