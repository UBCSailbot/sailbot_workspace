# Dev Container

## Features

- Starts from the [dev](./dev/README.md) image
- Copies the user configuration files in `.devcontainer/config/` to the container's home directory
    - See [config's README](./config/README.md) for more details
- Adds run arguments, container environment variables, VS Code extensions, and volumes
    - See [dev's README](./dev/README.md) for more details about the volumes

## How to Temporarily Add Packages

1. Uncomment optional section
2. Add the desired packages below the line `# Your package list here` with the format `<pkg1 name> <pkg2 name> ... \`
