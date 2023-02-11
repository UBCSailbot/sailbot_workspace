# Configuration Files

## Features

- Contains the VS Code workspace file which enables [multi-root workspaces](https://code.visualstudio.com/docs/editor/multi-root-workspaces)
- Can be used to copy your personal configuration files to the Dev Container

## How to use your personal configuration file in the Dev Container

1. Copy your personal configuration files (e.g., `.vimrc`) to this directory
    - To add aliases and functions to bash, put them in `.aliases.bash` and `.functions.bash`, respectively
    - Your Git configuration file, `~/.gitconfig`, is copied over by the dev container automatically
2. Ensure that the [dev image](../base-dev/base-dev.Dockerfile) installs the necessary packages (e.g., Vim for `.vimrc`)
3. Run the "Dev Containers: Rebuild Container" VS Code command
    - This will copy the files in this directory to the Dev Container's home directory

### Configuration

- The files in this directory are ignored by Git; to track a file, exclude its relative path in
  [the repository's `.gitignore` file](../../.gitignore)
    - For example, add `!.devcontainer/config/README.md` to track this file
- The files in this directory are copied to the Dev Container's home directory; to not copy a file, add its relative path
  to [the Dev Container's `.dockerignore` file](../.dockerignore)
    - For example, add `config/README.md` to not copy this file
