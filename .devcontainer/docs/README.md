# Docs Image

Used for running [our docs site](https://github.com/UBCSailbot/docs).

## Features

- Builds off the [`squidfunk/mkdocs-material`](https://hub.docker.com/r/squidfunk/mkdocs-material) image
- Installs the dependencies of the site located in the `docs/` folder of [`src/`](../../src/)
- When image is run, runs docs on port 8000

## How to run

1. Uncomment `"docker-compose.docs.yml",` in [`.devcontainer/devcontainer.json`](../devcontainer.json)
2. Run the "Dev Containers: Rebuild Container" VS Code command
