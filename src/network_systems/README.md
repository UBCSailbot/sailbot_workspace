# Network Systems

This repository contains the source code for all of UBC Sailbot's Network Systems programs. It is made to work as part
of [Sailbot Workspace](https://github.com/UBCSailbot/sailbot_workspace), and is **_not_** meant to be built as an
independent project.

## Setup

For comprehensive setup instructions, follow our [setup guide](https://ubcsailbot.github.io/sailbot_workspace/main/current/sailbot_workspace/setup/).

## Building

**Option A**: With sailbot_workspace open, invoke the VSCode `build` or `debug` task.

**Option B**: Run `/workspaces/sailbot_workspace/build.sh`

## Running

### ROS Launch

[Instructions found here.](https://ubcsailbot.github.io/sailbot_workspace/main/current/sailbot_workspace/launch_files/)

For example:

```bash
ros2 launch network_systems main_launch.py
```

This is the best option if multiple modules need to be run at once. Launch configurations are found under the
[config](config/) folder. These configurations define which modules to enable/disable and what parameters to use.

### ROS Run

If you just want to run a single module, then this is a direct and easy way to do it.

For example:

```bash
ros2 run network_systems example --ros-args -p enabled:=true
```

### Binary

Not recommended as you cannot pass ROS parameters, so modules may not work by default. Binaries for each module found
under [projects](projects/) can be found under
`/workspaces/sailbot_workspace/build/network_systems/projects/{module_name}/{module_name}`.

For example:

```bash
/workspaces/sailbot_workspace/build/network_systems/projects/example/example
```

## Testing

Unit tests specific to Network Systems is done using [GoogleTest](https://github.com/google/googletest). Unit tests
are defined per module. For example, under [projects/example/test/](projects/example/test/test_cached_fib.cpp).

### Run All Tests

**Option A**: With sailbot_workspace open, invoke the VSCode `test` task.

**Option B**: Under the sailbot_workspace directory, run `/workspaces/sailbot_workspace/scripts/test.sh`

Both options will run all of UBC Sailbot's tests, including those from other projects. More often than not, this is
unnecessary.

### Run and Debug Specific Tests

This is the preferred way to run and debug tests. When you open a test source file like
[the example's](projects/example/test/test_cached_fib.cpp), there will be green arrows next to each `TEST_F` macro.
Clicking a double green arrow runs a test suite, while clicking single green arrow runs one unit test. Right clicking
either arrow will open a prompt with a debug test option. When running a test via the debug option, we can set
breakpoints and step through our code line by line to resolve issues.

This convenient testing frontend is thank's to the
[TestMate extension](https://marketplace.visualstudio.com/items?itemName=matepek.vscode-catch2-test-adapter).

**Warning**: Large failing tests can crash VSCode. If this happens, either lower the size of the tests (ex. reduce
the number of iterations) or [run the test binary directly](#run-test-binaries).

### Run Test Binaries

Test binaries for each module found under projects can be found under
`/workspaces/sailbot_workspace/build/network_systems/projects/{module_name}/test_{module_name}`.

For example:

```bash
/workspaces/sailbot_workspace/build/network_systems/projects/example/test_example
```
