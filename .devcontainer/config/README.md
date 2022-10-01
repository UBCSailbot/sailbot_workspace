# Configuration Files

## Features

- Contains the VS Code workspace file which enables [multi-root workspaces](https://code.visualstudio.com/docs/editor/multi-root-workspaces)
- Can be used to get your personal configuration files onto the dev container

## How to Use Personal Configuration Files in the Dev Container

1. Copy your personal configuration files (e.g., `.vimrc`) to this directory
to add them to the dev container's home directory.
    - To add aliases and functions to bash, put them in `.aliases.bash` and `.functions.bash`, respectively
    - Your Git configuration file `~/.gitconfig` is copied over by the dev container automatically
2. Ensure that the [dev image](../dev/Dockerfile) installs the necessary packages (e.g., `vim` for `.vimrc`)

## Notes

- Files that begin with `.` in this directory are ignored by Git, so that your personal configuration files don't get added
