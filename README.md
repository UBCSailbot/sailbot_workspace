# Sailbot Workspace

This repository will get you set up to develop UBCSailbot's software on VS Code. It is based on athackst's
[vscode_ros2_workspace](https://github.com/athackst/vscode_ros2_workspace).
See their [write-up](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how
this workspace functions.

## Setup

This workspace can be installed on most operating systems, but it performs the best on [Ubuntu](https://ubuntu.com/download/desktop).

1. Install prerequisites
    - [docker](https://docs.docker.com/engine/install/)
        - For Windows, the WSL 2 backend for Docker Desktop is recommended
    - [vscode](https://code.visualstudio.com/)
    - [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

2. Clone this repository

    ```
    git clone https://github.com/UBCSailbot/sailbot_workspace.git
    ```

3. Open it in VS Code

    ```
    code sailbot_workspace
    ```

4. Open it in a container
    1. Make sure that Docker is running
    2. When you open it for the first time, you should see a little popup that asks you if you would like to open it in
       a container. Say yes!
    3. If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up
       the container dialog
        - In the dialog, select "Remote Containers: Reopen in container"
    4. VSCode will build the dockerfile inside of `.devcontainer` for you. If you open a terminal inside VSCode
       (Terminal > New Terminal), you should see that your username has been changed to `ros`, and the bottom left green
       corner should say "Dev Container"

5. Import the ROS packages and install their dependencies by running the "setup" task

## Run

1. Source the relevant overlay in the terminal
    - ROS 2: `srcnew`
    - ROS 1: `srcraye`

2. Build (this step might not be necessary if there are no changes made to C++ or custom msg nodes)
    - ROS 2: run the "Build" VS Code task, which has the keyboard shortcut `CTRL+SHIFT+B`
    - ROS 1: `roscd` then `catkin_make`

3. Run the ROS program
    - ROS 2: `ros2 run ...` or `ros2 launch ...`
    - ROS 1: `rosrun ...` or `roslaunch ...`

## Features

### Style

ROS2-approved formatters are included in the IDE.  

- **c++** uncrustify; config from `ament_uncrustify`
- **python** autopep8; vscode settings consistent with the [style guide](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/)

### Tasks

There are many pre-defined tasks, see [`.vscode/tasks.json`](.vscode/tasks.json) for a complete listing.
Bring up the task menu by typing "Tasks: Run Task" in the command pallete, or creating a keyboard shortcut for `workbench.action.tasks.runTask`.

### Debugging ([WIP](https://github.com/UBCSailbot/sailbot_workspace/issues/6))

This repository has debug configurations for Python files and Cpp programs.
See [`.vscode/launch.json`](.vscode/launch.json) for configuration details.
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
