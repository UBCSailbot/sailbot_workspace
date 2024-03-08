# Development Workflow

## 1. Open Sailbot Workspace

Once you have [set up Sailbot Workspace](setup.md), you can open it by opening a new VS Code window and selecting:

```
File > Open Recent > /workspaces/sailbot_workspace/.devcontainer/config/sailbot_workspace (Workspace) [Dev Container: Sailbot Workspace]
```

??? tip "Another way to open Sailbot Workspace on Windows"

    1. Pin VS Code to the taskbar
    2. Right-click VS Code in the taskbar and pin `sailbot_workspace (Workspace) [Dev Container]`

    Then you can open Sailbot Workspace by selecting it from the "Pinned" section of the VS Code taskbar icon's
    right-click menu.

## 2. Update Sailbot Workspace

Sailbot Workspace is still in active development, check out its [recent releases](https://github.com/UBCSailbot/sailbot_workspace/releases){target=_blank}
and [commit history](https://github.com/UBCSailbot/sailbot_workspace/commits/main){target=_blank}.
If there are new features or bug fixes that you want to try, you will need to update your local version of Sailbot Workspace:

1. Switch Sailbot Workspace to the main branch if you aren't in it already

    ??? tip "If you running Git commands in the CLI, make sure that you are in the correct repository"

        Sailbot Workspace contains other repositories in the `src/` directory, so if you are in one of its subdirectories
        you may be in the wrong repository.

        To check which repository you are in, run `git remote -v`; if its output contains `sailbot_workspace`,
        you are good to go.
        If not, you can navigate the root directory of the Sailbot Workspace repository with `cd $ROS_WORKSPACE`,
        or open a new terminal in its root directory with ++ctrl+shift+grave++ then ++enter++.

    - If you are unable to switch branches because you have uncommitted changes, stash them

2. Pull the latest changes
    - If you stashed your uncommitted changes, pop them
3. If prompted, rebuild the Dev Container

    ??? question "When does the Dev Container need to be rebuilt?"

        To apply the modifications to its configuration files in `.devcontainer/` that occurred since it was last built.

        VS Code will prompt you to rebuild when `devcontainer.json`, `Dockerfile`, or `docker-compose*.yml`.
        These file may be modified if you:

        - Pull the lastest changes of a branch
        - Switch branches
        - Update a file in `.devcontainer/` yourself

        However, there may be changes to the Dev Container that VS Code can't detect.
        To rebuild it yourself, run the `Dev Containers: Rebuild Container` VS Code command.

4. If you aren't working in any other branches,
   run the `setup` task to switch the branches of all sub-repositories to their default specified in `src/polaris.repos`
   and pull their latest changes
5. If you want to run our docs, website, or other optional programs, see [How to run optional programs](./how_to.md#run-optional-programs){target=_blank}

## 3. Make your changes

We make changes to our software following our [GitHub development workflow](https://ubcsailbot.github.io/docs/reference/github/workflow/overview/){target=_blank}.
Of particular relevance is the [Developing on Branches page](https://ubcsailbot.github.io/docs/reference/github/workflow/branches/){target=_blank}.

!!! tip "Git interfaces"

    One way to interface with Git is through CLI commands. However, you may find it faster to use
    [VS Code's interface](https://code.visualstudio.com/docs/sourcecontrol/overview){target=_blank},
    especially when working with multiple repositories.

Things to note when making changes:

- When C++ or Python files are saved, you may notice that some lines change. We use formatters to help fix lint errors;
  not all lint errors can be fixed by formatters, so you may have to resolve some manually
- When changing a package's source files, you likely should update its test files accordingly

## 4. Build your changes

!!! info ""

    Revamped in [:octicons-tag-24: v1.2.0](https://github.com/UBCSailbot/sailbot_workspace/releases/tag/v1.2.0){target=_blank}

In general, changes need to be built before they can be run. You can skip this step if you only modified Python source
or test files (in `python_package/python_package/` or `python_package/test`, respectively), or are running a launch type
launch configuration.

1. Depending on which packages you modified, run the `Build All` or `Build Package` task
    1. Unless you want to run `clang-tidy`, use the `-q` build argument (default) for quicker build times

## 5. Verify your changes

!!! info ""

    Revamped in [:octicons-tag-24: v1.2.0](https://github.com/UBCSailbot/sailbot_workspace/releases/tag/v1.2.0){target=_blank}

??? tip "Running GUI applications on macOS"

    If you want to run GUI applications on macOS, ensure that XQuartz is running.

### Lint and Test

Run lint and test tasks to make sure you changes will pass our CI:

- `ament lint`
- For C++ packages, `clang-tidy`
- `test`

In addition to VS Code tasks, the :fontawesome-solid-flask: **Testing** tab on the VS Code primary sidebar
contains individual tests. One can run specific unit tests by clicking the :material-play-outline: **Run Test**
icon beside the test name.

<img
    alt="VS Code Test Tab" src="../../../assets/images/sailbot_workspace/workflow/vscode_testing_tab.png"
    style="display: block; margin-left: auto; margin-right: auto; width: 50%;"
/>

### Run a Package

To verify that your changes do what you expect, you may want to run the package you modified. The run commands for each
package should be documented in their READMEs, but in general they can be run using a CLI or VS Code command:

=== ":octicons-command-palette-16: CLI"
    - Launch files:
        - `ros2 launch <package> <launch file>`
        - `ros2 launch <path to launch file>`
    - Nodes:
        - `ros2 run <package> <executable>`

    ??? tip "CLI features"

        There are many commands that can be autocompleted in the terminal.
        Take advantage of this so that you run commands faster and memorize less syntax.
        If there is only one possibility, pressing tab once will complete it.
        If there is more than one possibility, pressing tab again will list them out.

        Some tab completion use cases:

        - View available commands: lists all `ros2` commands

            ```console
            $ ros2 <tab><tab>
            action                          extension_points                multicast                       security
            bag                             extensions                      node                            service
            ...
            ```

        - Complete commands: runs `ros2 launch local_pathfinding main_launch.py`

            ```console
            $ ros2<tab>la<tab>loc<tab>m<tab>
            ```

        - Navigate to directories: runs `cd .devcontainer/config` from the root directory of Sailbot Workspace

            ```console
            $ cd .d<tab>c<tab>
            ```

        Furthermore, navigate past commands with ++arrow-up++ and ++arrow-down++ and search through them with ++ctrl+r++.

=== ":material-microsoft-visual-studio-code: VS Code"
    - Launch files: `ROS: Run a ROS launch file (roslaunch)`
    - Nodes: `ROS: Run a ROS executable (rosrun)`

For more information on launch file use in our system, see [this page](./launch_files.md){target=_blank}.

### Run the System

To verify that you didn't break anything, you may want to run the entire system. See
[Invoking Launch Files](./launch_files.md#invoking-launch-files) for more information
on running the system.

### Debugging

Debug your changes if they aren't behaving how you expect by setting breakpoints and running one of our launch
configurations in the **Run and Debug** tab on the VS Code primary sidebar. The launch configuration types are:

- Launch: runs the desired launch file or executable
    - For launch files, `ROS: Launch`
    - For C++ executables, `C++ (GDB): Launch`
- Attach: attaches to a running executable
    - `ROS: Attach`

## Troubleshooting

If you are having some trouble running our software, here are some things you can try:

- Build from scratch
    1. Run the `clean` task to delete C++ generated files
    2. Run the `purge` task to delete ROS generated files
    3. Run the `Build All` task to rebuild
- Rebuild the Dev Container: run the `Dev Containers: Rebuild Container` VS Code command
- Reload VS Code: run the `Developer: Reload Window` VS Code command
- Delete Docker files

    ??? tip "Running Docker CLI commands on Windows"

        On Windows, Docker CLI commands should be run in the Ubuntu terminal while Docker Desktop is running.

    - Run `docker system prune` to remove all unused containers, networks, and dangling and unreferenced images
        - Add `--all` to additionally remove unused images (don't have a container associated with them)
        - Add `--volumes` to additionally remove volumes (makes Bash history and ROS logs persist across containers)
    - Run `docker rmi -f $(docker images -aq)` to remove all images

## Performance Issues

If you are not satisfied with the performance of Sailbot Workspace, here are some things you can try:

- Free up memory: close programs that you aren't using
- Free up disk space: permanently delete large programs and files that you don't need anymore
- Run Sailbot Workspace in a GitHub Codespace
    - In a codespace with 8GB of RAM, building all packages from scratch with the `-q` argument takes about a minute.
    If your computer takes longer than, or you want to free up memory and disk space, you can
    [setup Sailbot Workspace in a GitHub Codespace](./setup.md#setup-sailbot-workspace-in-a-github-codespace){target=_blank}
- If you are running Sailbot Workspace on Windows, dual boot Ubuntu and run Sailbot Workspace there
    - Sailbot Workspace performs worse on Windows than bare metal Linux because it uses Docker, which is not natively supported.
    - Here is a guide to dual boot the operating systems we recommend: [How to Dual Boot Ubuntu 22.04 LTS and Windows 11](https://www.linuxtechi.com/dual-boot-ubuntu-22-04-and-windows-11/){target=_blank}
        - We recommend allocating at least 50 GB to Ubuntu to leave some wiggle room for Docker
        - The process is similar for other Ubuntu and Windows versions,
          but feel free to search for a guide specific to the combination you want to dual boot
        - Since Sailbot Workspace uses Docker, it should be able to run on any Linux distribution, not just Ubuntu.
          However, we may not be able to provide support if you encounter any difficulties with this
