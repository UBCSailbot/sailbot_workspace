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

### Temporarily add packages

If a task requires you to add packages, you can test it in the Sailbot Workspace image:

1. Uncomment the section in [`Dockerfile`](./Dockerfile) that installs additional packages
2. Add the desired packages below the line `# Your package list here` with the format `<pkg1 name> <pkg2 name> ... \`
3. Run the "Dev Containers: Rebuild Container" VS Code command

Before merging in the PR, you should migrate the package installations to upstream images:

- [`base`, `local-base`, `dev`](./base-dev/)
- [`pre-base`](./pre-base/)
