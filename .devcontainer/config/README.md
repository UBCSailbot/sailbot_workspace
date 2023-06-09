# Configuration Files

## Features

- Contains the VS Code workspace file which enables [multi-root workspaces](https://code.visualstudio.com/docs/editor/multi-root-workspaces)
- Can be used to copy your personal configuration files to the Dev Container

## Configuration

- The files in this directory are ignored by Git; to track a file, exclude its relative path in
  [the repository's `.gitignore` file](../../.gitignore)
    - For example, add `!.devcontainer/config/README.md` to track this file
- The files in this directory are copied to the Dev Container's home directory; to not copy a file, add its relative path
  to [the Dev Container's `.dockerignore` file](../.dockerignore)
    - For example, add `config/README.md` to not copy this file
