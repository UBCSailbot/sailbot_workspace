# Website Image

Used for running [our website](https://github.com/UBCSailbot/website).

## Features

- Builds off the [`mcr.microsoft.com/vscode/devcontainers/javascript-node`](https://hub.docker.com/_/microsoft-vscode-devcontainers)
  image
- Installs the dependencies of the site located in the `website/` folder of [`src/`](../../src/)

## How to run

1. Uncomment `"docker-compose.website.yml",` in [`devcontainer.json`](./devcontainer.json)
2. If you want to access the database, uncomment `"docker-compose.db.yml",` as well
3. Run the "Dev Containers: Rebuild Container" VS Code command
