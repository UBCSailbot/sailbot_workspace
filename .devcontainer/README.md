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

- Builds off the [`dev`](./base-dev/base-dev.Dockerfile) image
- Copies the user configuration files in [`.devcontainer/config/`](./config/) to the container's home directory
    - See [config's README](./config/README.md) for more details

### How to temporarily add packages

1. Uncomment the section in [`.devcontainer/Dockerfile`](./Dockerfile) that installs additional packages
2. Add the desired packages below the line `# Your package list here` with the format `<pkg1 name> <pkg2 name> ... \`
3. Run the "Dev Containers: Rebuild Container" VS Code command
