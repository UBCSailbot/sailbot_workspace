# How-To's

## Run VS Code commands, tasks, and launch configurations

!!! tip "MacOS keyboard shortcuts"

    For keyboard shortcuts on MacOS, substitute ++ctrl++ with ++cmd++.

VS Code commands can be run in the Command Palette.
Open the Command Palette from the `View` menu or with ++ctrl+shift+p++.

Tasks can be run using the `Tasks: Run Task` VS Code command. Build tasks can be run with ++ctrl+shift+b++.

Launch configurations can be run from the [Run and Debug view](https://code.visualstudio.com/docs/editor/debugging#_run-and-debug-view){target=_blank}.

You can also run VS Code commands, tasks, launch configurations, and much more by typing their prefixes
into an empty Command Palette. Open an empty Command Palette with ++ctrl+p++ or by clicking the box in the center
of the title bar. See the list below for some prefixes and their functions.
For prefixes that are words, you will have to append a space to them to bring up their functions.

- Nothing: files
- `>`: VS Code commands
- `task`: tasks
- `debug`: launch configurations
- `?`: list all prefixes and their functions

## Work with containerized applications

!!! info ""

    New in [:octicons-tag-24: v1.1.0](https://github.com/UBCSailbot/sailbot_workspace/releases/tag/v1.1.0){target=_blank}

We have containerized the following applications for a variety of reasons:

- [MongoDB database](https://www.mongodb.com/){target=_blank}
- [Docs site](https://github.com/UBCSailbot/docs){target=_blank}
- [Website](https://github.com/UBCSailbot/website){target=_blank}

### Running containerized applications

In the first section of `dockerComposeFile` of `.devcontainer/devcontainer.json`, there is a list of files:
each file contains the configuration for one or more applications.

The ones that are commented out are not run. To run them:

1. Uncomment the Docker Compose file(s) that the application(s) you desire to run are defined in
    - Programs that are defined in the uncommented Docker Compose files will be started and stopped with Sailbot Workspace
2. Run the `Dev Containers: Rebuild Container` VS Code command to restart Sailbot Workspace

To stop running them:

1. Comment out the corresponding Docker Compose file
2. Stop the application's container: see [Managing containerized applications](#managing-containerized-applications)

### Viewing MongoDB data

Connect the [MongoDB VS Code extension](https://www.mongodb.com/products/vs-code){target=_blank} to the running database:
[Create a Connection for Deployment](https://www.mongodb.com/docs/mongodb-vscode/connect/#create-a-connection-to-a-deployment){target=_blank}

- Use the default methods: "Paste Connection String" and "Open from Overview Page"
- Our database's connection string is `mongodb://localhost:27017`
- See the [MongoDB VS Code extension docs](https://www.mongodb.com/docs/mongodb-vscode/){target=_blank} for how
  to use it to navigate or explore the database

### Opening Docs or Website

Docs runs on port 8000 and Website 3005. You can see them in your browser at `localhost:<port>`. To open them using VS Code:

1. Run the `Ports: Focus on Ports View` VS Code command
2. Open the site by hovering over its local address and clicking either "Open in Browser" or "Preview in Editor"
    - The local address of Docs is the line with a port of 8000
    - The local address of Website is the line with a port of 3005

!!! tip "Turn off auto saving"

    Changes made to their files are loaded when they are saved, so **if Auto Save is on, turn it off**
    so that the Docs/Website servers aren't continuously reloading. Auto Save is on by default in GitHub Codespaces

### Managing containerized applications

Each application runs in a Docker container. Containers can be managed using Docker Desktop or CLI commands:

- View Sailbot Workspace containers

    === ":material-docker: Docker Desktop"

        1. Select "Containers" in the top right
        2. Expand "sailbot_workspace_devcontainer"
            - The "Status" column shows whether a container is running or not

    === ":octicons-terminal-24: CLI Commands"

        ```
        docker ps -a
        ```

        - Sailbot Workspace containers should be named something like `sailbot_workspace_devcontainer-<application>-<number>`
        - The `STATUS` column shows whether a container is running or not

- View a container's logs, the output of the container (including errors that caused it to stop)

    === ":material-docker: Docker Desktop"

        1. Click on a container
        2. Navigate to the "Logs" view if not already on it

    === ":octicons-terminal-24: CLI Commands"

        ```
        docker logs <container>
        ```

- Start a container that is not running

    === ":material-docker: Docker Desktop"

        1. Click start :material-play:

    === ":octicons-terminal-24: CLI Commands"

        ```
        docker start <container>
        ```

- Stop a container that is running

    === ":material-docker: Docker Desktop"

        1. Click stop :material-stop:

    === ":octicons-terminal-24: CLI Commands"

        ```
        docker stop <container>
        ```

## Manage software packages

!!! warning "Why can't I just install the dependencies myself in the command line interface with `pip` or `apt`?"

    Although this will temporarily work, installing apt and/or Python dependencies directly in sailbot workspace using
    the commandline interface will not persist between container instances. The dependencies will need to be manually
    installed every single time you create a new instance of sailbot workspace, which is not feasible when we start to
    use many dependencies at once.

    Of course, one could also install dependencies inside the sailbot workspace Docker images to allow such dependencies
    to persist across container instances. However, putting dependencies inside `package.xml` distinguishes between
    what dependencies are needed for ROS packages and what dependencies are needed for infrastructure purposes.

### Add apt or python dependencies to ROS packages

If running your ROS packages requires external dependencies from an apt repository or python package, **one** of the following
tags should be added to the `package.xml` file in the root directory of the ROS package:

```xml
<depend>ROSDEP_KEY</depend>
<build_depend>ROSDEP_KEY</build_depend>
<build_export_depend>ROSDEP_KEY</build_export_depend>
<exec_depend>ROSDEP_KEY</exec_depend>
<test_depend>ROSDEP_KEY</test_depend>
```

- Learn what each tag is used for [here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#id4){target=_blank}.

- Replace `ROSDEP_KEY` with the rosdep key for the dependency, which can be found online.
    - Use the key associated with **ubuntu** since sailbot workspace uses Ubuntu, or **debian** which Ubuntu is based on
    - Do not include the square brackets in `package.xml`

    === ":material-ubuntu: Apt Dependencies"
        - Rosdep keys for apt repositories can be found [here](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml){target=_blank}

    === ":material-language-python: Python Dependencies"
        - Rosdep keys for python packages can be found [here](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml){target=_blank}
        - Since we use Python 3, look for the packages that start with `python3-` (`python-` is usually for Python 2)

- If there isn't rosdep key for the dependency, you can add your own to `custom-rosdep.yaml`
  in the root directory of the ROS package

After completing these steps, [run the `setup` task](#run-vs-code-commands-tasks-and-launch-configurations) and the
desired dependencies should be installed. ROS uses a dependency management utility, rosdep, to handle the installation
of dependencies. In addition to runtime dependencies, rosdep also handles dependencies for build time, dependencies for
testing, sharing dependencies between ROS packages, and more.
See the [ROS documentation on rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html){target=_blank}
to learn more.

### Add dependencies to a Docker image

There are a couple cases where you would want to add dependencies to a Docker image instead of ROS package:

1. The dependency is not used to build/run/test a ROS package
2. There is no apt or pip package for your dependency so you have to build from source

To verify your changes, you can add them to `.devcontainer/Dockerfile` then
run the `Dev Containers: Rebuild Container` VS Code command. Once verified, migrate the changes to one of the upstream
[images](./docker_images.md){target=_blank}: `base`, `local-base`, `dev`, or `pre-base`.

## Enable GitHub Copilot in Sailbot Workspace

[^1]: [GitHub Copilot Quickstart Guide](https://docs.github.com/en/copilot/quickstart){target=_blank}

GitHub Copilot is an AI paired programming tool that can help you accelerate your development by providing suggestions
for whole lines or entire functions inside your editor.[^1] To enable GitHub Copilot:

1. [Apply to GitHub Global Campus as a student](https://docs.github.com/en/education/explore-the-benefits-of-teaching-and-learning-with-github-education/github-global-campus-for-students/apply-to-github-global-campus-as-a-student){target=_blank}
to use GitHub Copilot and get other student benefits for free. It may take a few days for your student status to be
verified. In the meantime, you can still continue with the next steps. However, you will need to use the GitHub Copilot
free trial until your account is verified.

2. [Sign up for GitHub Copilot for your personal account](https://docs.github.com/en/copilot/quickstart#signing-up-for-github-copilot-for-your-personal-account){target=_blank}.
If it offers a free trial, then take it. You should see a page telling you that you can use GitHub Copilot for free
(if you have a verified student account).

3. Uncomment the `github.copilot` extension in `.devcontainer/devcontainer.json` and run the
   `Dev Containers: Rebuild Container` VS Code command

4. Sign into your GitHub account in VS Code. The GitHub Copilot extension should automatically prompt you to sign into
your account if you are not already.

    ??? warning "VS Code is not prompting me to sign into my account"
        You may already be signed in into your GitHub account. You can check by clicking on the :octicons-person-16:
        **Accounts** icon in the bottom-left corner in VS Code and verify that you see your GitHub account.

        If you do not see your account, you can get the sign in prompt by trying:

        - Reloading the VS Code window: ++ctrl+shift+p++ and select `Developer: Reload Window`
        - Rebuilding the devcontainer: ++ctrl+shift+p++ and select `Dev Containers: Rebuild Container`
        - If using a Mac, use ++cmd++ instead of ++ctrl++

5. If all the previous steps were done correctly, you should see the :octicons-copilot-48: **GitHub Copilot** icon in
the bottom-right corner of VS Code without any error messages. For more information on how to use Copilot and a tutorial,
refer to:

    - [The GitHub Copilot Getting Started Guide](https://docs.github.com/en/copilot/getting-started-with-github-copilot){target=_blank}
    - [Configuring GitHub Copilot in your Environment](https://docs.github.com/en/copilot/configuring-github-copilot/configuring-github-copilot-in-your-environment){target=_blank}

## Use your dotfiles

Dotfiles are configuration files for various programs.[^2]

??? info "More about dotfiles"

    - They are called dotfiles because their filenames start with a dot (`.`)
    - On Linux and MacOS, files and directories that begin with a dot are hidden by default
    - To list dotfiles using the `ls` command, specify the `-a` argument: `ls -a`

Dotfiles that are commonly modified include:

- Bash: `~/.bashrc`
- Git: `~/.gitconfig`
- Vim: `~/.vimrc`

To use your dotfiles:

1. Ensure that the `base`, `local-base`, or `dev` [image](./docker_images.md){target=_blank}
   installs the programs that the dotfiles correspond to
2. Copy the dotfiles to the `.devcontainer/config/` directory.
   If a dotfile is located in a child directory, you will have to created it.
   For example, if a dotfile's path is `~/.config/ex_dotfile`, you will need to copy it to `.devcontainer/config/.config/ex_dotfile`

    !!! warning "Special cases"

        - `~/.gitconfig`: there is no need copy your Git dotfile, as Dev Containers do this automatically
        - `~/.bashrc`: don't copy your Bash dotfile, as it would override the one created in the `dev` image.
        Instead, add your bash configuration `.aliases.bash` or `.functions.bash` in the config directory, as these are sourced
        by the created Bash dotfile.

3. Run the `Dev Containers: Rebuild Container` VS Code command

[^2]: [Dotfiles – What is a Dotfile and How to Create it in Mac and Linux](https://www.freecodecamp.org/news/dotfiles-what-is-a-dot-file-and-how-to-create-it-in-mac-and-linux/){target=_blank}

## Run Raye's software

Raye was our previous project. Her software can be run in the `raye` branch:

1. Switch to the `raye` branch: `git switch raye`
2. Rebuild the Dev Container: run the `Dev Containers: Rebuild Container` VS Code command
3. If you want to run [Raye's local pathfinding visualizer](https://github.com/UBCSailbot/raye-local-pathfinding?tab=readme-ov-file#visualizing-the-simulation){target=_blank},
   complete [step 2 of the setup instructions](./setup.md#2-setup-x11-forwarding){target=_blank}

!!! warning "`raye` branch disclaimers"

    1. Since `raye` (and Raye's codebase in general) is not in active development, it may not be 100% functional
       or contain all the features in `main`
    2. `raye` is more memory intensive than `main` because the parent image of its Dev Container is much larger;
       this may lead to worse performance

### Build Raye's ROS packages

To build Raye's ROS packages, run the following commands:

```bash
roscd
catkin_make
```

### Run packages from different workspaces

The `raye` branch has two ROS workspaces: one for Raye and one for the new project.
To run ROS packages, you will have to source the overlay of the workspace that it is in:

=== "New Project"

    ```
    srcnew
    ```

=== "Raye"

    ```
    srcraye
    ```

Then you can run launch files or package-specific executables in that workspace with:

=== "New Project"

    `ros2 launch ...` or `ros2 run ...`, respectively.

=== "Raye"

    `roslaunch ...` or `rosrun ...`, respectively.

### Raye's known issues

!!! bug "Run commands for Raye packages are very slow"

    On non-Ubuntu-based Linux operating systems, Run commands for Raye packages may take a long time to start-up.
    This is because the system has trouble resolving the local hostname.

    To resolve this bug, run the commands below in the Dev Container:

    ```bash
    echo 'export ROS_HOSTNAME=localhost' >> ~/.bashrc
    echo 'export ROS_MASTER_URI=http://localhost:11311' >> ~/.bashrc
    ```
