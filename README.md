# Sailbot Workspace

This repository will get you set up to develop UBCSailbot's software on VS Code. It is based on athackst's
[vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace).
See their [write-up](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more
in-depth look on how this workspace functions.

## Features

### Style

ROS2-approved formatters are included in the IDE.

- **C++** uncrustify; config from `ament_uncrustify`
- **Python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)

### Tasks

There are many pre-defined tasks. See
[our workspace file](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/config/sailbot_workspace.code-workspace)
for a complete listing. Bring up the task menu by typing "Tasks: Run Task" in the command pallete, or creating a keyboard
shortcut for `workbench.action.tasks.runTask`.

### Debugging ([WIP](https://github.com/UBCSailbot/sailbot_workspace/issues/6))

This repository has debug configurations for Python files and C++ programs. See
[our workspace file](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/config/sailbot_workspace.code-workspace)
for configuration details.
Bring up the debug configurations menu by typing "debug " in the command palette without the ">" prefix, or select one
from the Run and Debug view.

### Continuous Integration

This repository also has continuous integration that lints and tests our code.
See [`.github/workflows/`](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.github/workflows)
for the configuration files.

### Customization

This repository supports user-specific configuration files. To set this up see
[how to use your personal configuration file in the Dev Container](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.devcontainer/config#how-to-use-your-personal-configuration-file-in-the-dev-container).

## Run Raye's Software

Raye was our previous project. Its software can be run in the [`raye` branch](https://github.com/UBCSailbot/sailbot_workspace/tree/raye).
The initial differences between the `main` and `raye` branches are summarized in
[this PR](https://github.com/UBCSailbot/sailbot_workspace/pull/61).

## Documentation

Further documentation, including setup and run instructions, can be found on [our Docs website](https://ubcsailbot.github.io/docs/current/sailbot_workspace/overview/).
