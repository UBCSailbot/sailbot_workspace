# Sailbot Workspace

[![Test](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/test.yaml/badge.svg)](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/test.yaml)
[![Lint](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/lint.yaml/badge.svg)](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/lint.yaml)

This repository will get you set up to develop UBCSailbot's software on VS Code. It is based on athackst's
[vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace).
See their [write-up](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how
this workspace functions.

## Setup

This workspace can be set up on most operating systems, but it performs the best and requires the least setup on
Ubuntu and [its derivatives](https://distrowatch.com/search.php?basedon=Ubuntu).

1. Install prerequisites
    - For Windows, [WSL](https://learn.microsoft.com/en-us/windows/wsl/about)
        - Run these commands in an *administrator* PowerShell window

            ```
            wsl --install --distribution Ubuntu
            wsl --set-default Ubuntu
            wsl --set-version Ubuntu 2
            ```

    - [Docker](https://docs.docker.com/get-started/overview/)
        - [Install Docker](https://docs.docker.com/engine/install/)
            - For Windows or MacOS, install Docker Desktop
                - For Windows, use the WSL 2 backend for Docker Desktop
            - For Linux, install Docker Engine
        - For Linux
            - [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)
            - [Configure Docker to start on boot](https://docs.docker.com/engine/install/linux-postinstall/#configure-docker-to-start-on-boot)
    - [VS Code](https://code.visualstudio.com/)
        - [Install VS Code](https://code.visualstudio.com/download)
        - [Install VS Code Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)

2. For Windows and MacOS, additional configuration to run GUI applications
    - For Windows 11, GUI applications work without additional configuration,
      *but if you upgraded from Windows 10 make sure to update the WSL kernel:* `wsl --update`
      in an *administrator* PowerShell window
    - For Windows 10
        - Follow [this guide](https://techcommunity.microsoft.com/t5/windows-dev-appconsult/running-wsl-gui-apps-on-windows-10/ba-p/1493242)
          up to setting the `DISPLAY` variable, exclusive
        - Put ``export DISPLAY="\`grep nameserver /etc/resolv.conf | sed 's/nameserver //'\`:0"`` into `~/.bashrc`
          of the WSL file system
        - If VS Code is open, restart it
    - For MacOS
        - Follow [this guide](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088) to setup XQuartz
        - Additional XQuartz configuration
            - `cp /opt/X11/etc/X11/xinit/xinitrc ~/.xinitrc`
            - Add `xhost +localhost` to `~/.xinitrc` after its first line
            - If XQuartz is open, restart it
        - Zsh configuration
            - Add `export MAC_DOCKER_LOCALHOST="docker.for.mac.host.internal"` and `export DISPLAY=:0` to `~/.zshrc`
            - If VS Code is open, restart it

3. Clone this repository

    ```
    git clone https://github.com/UBCSailbot/sailbot_workspace.git
    ```

    - For Windows, clone the repository in the WSL filesystem, for example `~/sailbot` in the Ubuntu WSL terminal

4. Open it in VS Code

    ```
    code sailbot_workspace
    ```

5. Open it in a container
    1. Make sure that Docker is running
    2. When you open it for the first time, you should see a little popup that asks you if you would like to open it in
       a container. Say yes!
    3. If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up
       the container dialog
        - In the dialog, select "Reopen in container"
    4. VSCode will build the dockerfile inside of `.devcontainer` for you. If you open a terminal inside VSCode
       (Terminal > New Terminal), you should see that your username has been changed to `ros`, and the bottom left green
       corner should say "Dev Container"

6. Import the ROS packages and install their dependencies by running the "setup" task

## Run

1. For Windows 10 and MacOS, if you want to run something with a GUI
    - For Windows 10, open the XLaunch configuration file
    - For MacOS, start XQuartz

2. Source the relevant overlay in the terminal
    - ROS 2: `srcnew`
    - ROS 1: `srcraye`

3. Build (this step might not be necessary if there are no changes made to C++ or custom msg nodes)
    - ROS 2: run the "Build" VS Code task, which has the keyboard shortcut `CTRL+SHIFT+B`
    - ROS 1: `roscd` then `catkin_make`

4. Run the ROS program
    - ROS 2: `ros2 run ...` or `ros2 launch ...`
    - ROS 1: `rosrun ...` or `roslaunch ...`

## Features

### Style

ROS2-approved formatters are included in the IDE.  

- **c++** uncrustify; config from `ament_uncrustify`
- **python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)

### Tasks

There are many pre-defined tasks, see [our workspace file](.devcontainer/config/sailbot_workspace.code-workspace) for a
complete listing. Bring up the task menu by typing "Tasks: Run Task" in the command pallete, or creating a keyboard
shortcut for `workbench.action.tasks.runTask`.

### Debugging ([WIP](https://github.com/UBCSailbot/sailbot_workspace/issues/6))

This repository has debug configurations for Python files and Cpp programs.
See [our workspace file](.devcontainer/config/sailbot_workspace.code-workspace) for configuration details.
Bring up the debug configurations menu by typing "debug " in the command pallete without the ">" prefix, or select one
from the Run and Debug view.

### Continuous Integration

This repository also has continuous integration that lints and tests our code.
See [`.github/workflows/`](.github/workflows/) for the configuration files.

### Configured Terminal Commands and Aliases

| ROS 2 Command | ROS 1 Command | Function |
| ------------- | ------------- | -------- |
| `colcon_cd` | `roscd` | Navigate to ROS workspace |
| `srcnew` | `srcraye` | Source ROS workspace overlay |
