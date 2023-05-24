# Docs Image

Used for running [our docs site](https://github.com/UBCSailbot/docs).

## Features

- Builds off the [`squidfunk/mkdocs-material`](https://hub.docker.com/r/squidfunk/mkdocs-material) image
- Installs the dependencies of the site located in the `docs/` folder of [`src/`](../../src/)

## How to build

1. Uncomment `"docker-compose.docs.yml",` in [`devcontainer.json`](./devcontainer.json)
2. Run the `Dev Containers: Rebuild Container` command in the
   [VS Code command palette](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette)
