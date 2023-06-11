# Sailbot Workspace

[![Build Images](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/build-images.yml/badge.svg)](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/build-images.yml)
[![Tests](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/tests.yml/badge.svg)](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/tests.yml)

This repository will get you set up to develop UBCSailbot's software on VS Code. It is based on athackst's
[vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace).

## Features

An overview of Sailbot Workspace's features can be found below.
See [our docs site](https://ubcsailbot.github.io/docs/current/sailbot_workspace/run/) for how to use these features.

### Style

C++ and Python linters and formatters are integrated into Sailbot Workspace:

- ament_flake8
- ament_lint_cmake
- ament_xmllint
- black
- clang-tidy
- isort

The [ament linters](https://github.com/ament/ament_lint/tree/humble) are configured to be consistent with the
[ROS style guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).

### Dev Container

[Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers) enable us to use a
[Docker container](https://www.docker.com/resources/what-container/) as a fully-featured development environment
containing all our configuration and dependencies.
Our Dev Container configuration can be found in [`.devcontainer/`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer).

### Multi-Root Workspace

[Workspaces](https://code.visualstudio.com/docs/editor/workspaces) are VS Code instances that contain one or more folders.
Our workspace configuration file can be found at
[`.devcontainer/config/sailbot_workspace.code-workspace`](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/config/sailbot_workspace.code-workspace).

Our software spans many repositories: [software team repositories](https://github.com/orgs/UBCSailbot/teams/software-team/repositories).
[Multi-root workspaces](https://code.visualstudio.com/docs/editor/multi-root-workspaces)
make it easy to work with multiple repositories at the same time.
Our roots are defined in the `folders` section of our workspace file.

### Tasks

[Tasks](https://code.visualstudio.com/docs/editor/tasks) provide an alternative to memorizing the multitude of
CLI commands we use to setup, build, lint, test, and run our software. They are defined in `tasks` section of
our workspace file.

### Debugging

[Launch configurations](https://code.visualstudio.com/docs/editor/debugging#_launch-configurations)
have been created to debug our software. They are defined in the `launch` section of
our workspace file.

### Continuous Integration

[Actions](https://docs.github.com/en/actions/learn-github-actions/understanding-github-actions)
were used to build [our Docker containers](https://github.com/orgs/UBCSailbot/packages?repo_name=sailbot_workspace)
and lint and test our code the same way it is done locally in Sailbot Workspace on GitHub.
We use a [reusable workflow](https://docs.github.com/en/actions/using-workflows/reusing-workflows)
to create a single source of truth for our tests across all our repositories.
Our CI can be found in [`.github/workflows/`](https://github.com/UBCSailbot/sailbot_workspace/tree/main/.github/workflows).

### Customization

This repository supports user-specific configuration files. To set this up, see
[How to use your dotfiles](https://ubcsailbot.github.io/docs/current/sailbot_workspace/how_to/#use-your-dotfiles).

### Run Raye's Software

[Raye](https://www.ubcsailbot.org/discover-raye) was our previous project.
Her software can be run in the [`raye` branch](https://github.com/UBCSailbot/sailbot_workspace/tree/raye).
The initial differences between the `main` and `raye` branches are summarized in
[this PR](https://github.com/UBCSailbot/sailbot_workspace/pull/61).

## Documentation

Further documentation, including setup and run instructions, can be found on [our Docs website](https://ubcsailbot.github.io/docs/current/sailbot_workspace/overview/).
