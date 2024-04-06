# Website Image

Used for running [our website](https://github.com/UBCSailbot/sailbot_workspace/tree/main/src/website).

## Features

- Builds off the [`mcr.microsoft.com/vscode/devcontainers/javascript-node`](https://hub.docker.com/_/microsoft-vscode-devcontainers)
  image
- Installs MongoDB command line tools
- Installs the dependencies of the site located in the `website/` folder of [`src/`](../../src/)
- When image is run, runs website on port 3005
