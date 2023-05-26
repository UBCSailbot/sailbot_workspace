# Dev Container

## Features

- Services defined (in the Docker Compose files) are installed in their own containers
    - Services include [this repository](#sailbot-workspace-image), [docs](./docs/), and [website](./website/)
- Highly configurable through its configuration file, [`devcontainer.json`](./devcontainer.json)
    - Control which services are run
    - Define environment variables
    - Specify which VS Code extensions to install
    - And [much more](https://containers.dev/implementors/json_reference/)

## Sailbot Workspace image

### Sailbot Workspace features

- Builds off the [`dev`](./base-dev/) image
- Copies the user configuration files in [`config/`](./config/) to the container's home directory
    - See [config's README](./config/README.md) for more details
