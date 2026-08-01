# Network Systems

This repository contains the source code for all of UBC Sailbot's Network
Systems programs. It is made to work as part of
[Sailbot Workspace](https://github.com/UBCSailbot/sailbot_workspace), and is
**_not_** meant to be built as an independent project.

## Setup

<!-- markdownlint-disable-next-line MD013 -->
For comprehensive setup instructions, follow our [setup guide](https://ubcsailbot.github.io/sailbot_workspace/main/current/sailbot_workspace/usage/setup/).

## Building

**Option A**: With sailbot_workspace open, invoke the VSCode `build` or
`debug` task.

**Option B**: Run `/workspaces/sailbot_workspace/build.sh`

## Running

### Wind Direction Convention

`WindSensor.direction` uses the flow-toward convention in the boat frame: the
angle points where the air travels, 0° flows from bow toward stern, and values
increase clockwise. The CAN transceiver preserves the sensor angle while
normalizing its range and filtering its vector components; it does not apply a
180° source/flow conversion. Raw CAN wind sensors must therefore provide the
same flow-toward convention.

### CAN Log Replay

<!-- markdownlint-disable-next-line MD013 -->
`can_transceiver_node` can replay a recorded candump-style CSV log instead of talking to real or mocked CAN hardware, so the rest of the stack (`controller`, `local_pathfinding`) can be exercised against realistic, previously-recorded CAN traffic without a boat. This is what `mode:="can"` (see [global_launch's README](../global_launch/README.md)) uses; `development` and `sim` modes are unaffected and always use a self-looping mock file.

Replay is controlled by three ROS parameters on `can_transceiver_node`:

<!-- markdownlint-disable MD013 -->
- `can_replay_file`: path to a candump-style CSV log. Expected row format is
  `<ISO timestamp>,<elapsed seconds>,<iface>  <hex ID>  [<decimal length>]  <hex bytes>`,
  e.g. `2026-06-06T20:16:32.154049,5645.967,can0  041  [04]  5D 01 40 00`.
  Frames are timed and ordered using the timestamp column (not the elapsed
  seconds column, since combined logs from multiple sessions can have
  inconsistent elapsed bases). Rows the logger split mid-frame across two
  lines are reassembled automatically, and a one-line summary of any
  malformed or dropped rows is printed once the file finishes parsing. **This
  parameter is required when `mode:="can"`**; the node fails to start if it
  is empty.
- `can_replay_rate`: playback speed multiplier. `1.0` (default) replays at
  the log's original pace, `2.0` replays twice as fast, and any value `<= 0`
  replays as fast as possible with no pacing.
- `can_replay_loop`: `true` restarts from the beginning of the log once the
  last frame is replayed, instead of stopping. Defaults to `false`.
- `can_replay_synthesize_heading`: `true` derives a `RUDDER_DATA_FRAME`
  (0x050) from the undocumented 0x204 attitude frame, so replay publishes
  `/rudder`. See "Heading synthesis" below. Defaults to `false` in code, but
  is enabled in `globals.yaml` and `on_water_globals.yaml`.
<!-- markdownlint-enable MD013 -->

#### Heading synthesis

<!-- markdownlint-disable MD013 -->
Logs recorded before the e-compass began sending `RUDDER_DATA_FRAME` (0x050) contain no such frame at all, so a replayed stack never publishes `/rudder`. `local_pathfinding` treats a missing heading as an inactive input and **publishes desired heading with the sail disabled**, which makes replay useless for exercising navigation.

Those logs do still carry the heading, in an undocumented 16-byte attitude frame on ID `0x204` that also appears to hold pitch and roll. With `can_replay_synthesize_heading: true`, the replayer emits a synthetic `RUDDER_DATA_FRAME` after each such frame, decoded as a little-endian `uint16` at byte offset 6 in centidegrees.

> **Warning:** this decode is **inferred from log analysis, not confirmed by the firmware team**. Supporting evidence: the field spans exactly 0–35999 centidegrees, changes by a median of 0.28° between consecutive frames, and tracks GPS-derived course over ground with a median offset of −3.6° (90% of samples within 30°). The node logs a warning whenever synthesis is active. Treat the resulting `/rudder` data as **simulated** — it must not be relied on outside of log replay, and `0x204` should be confirmed with the firmware/electrical team (and added to the [CAN Frames wiki](https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1827176527/CAN+Frames)) before anything depends on it.
<!-- markdownlint-enable MD013 -->

`globals.yaml` and `on_water_globals.yaml` both set `can_replay_file` to the
sample log checked into
[`lib/can_log/combined_candump.csv`](lib/can_log/combined_candump.csv), so
`mode:="can"` works out of the box with the default (`globals.yaml`) config.
To replay a different log or change the pacing, either point `config` at a
custom yaml with your own `can_transceiver_node.ros__parameters`, or run
the node directly:

```bash
ros2 run network_systems can_transceiver --ros-args \
  -p enabled:=true -p mode:=can \
  -p can_replay_file:=/path/to/your/log.csv -p can_replay_rate:=0
```

NOTE:
In CAN mode, the local and remote transceivers will effectively run in development
mode. It is done like this to prevent running the satellite or the database when
we are simply testing it in our development environment.

### ROS Launch

<!-- markdownlint-disable-next-line MD013 -->
[Instructions found here.](https://ubcsailbot.github.io/sailbot_workspace/main/current/sailbot_workspace/reference/launch_files/)

For example:

```bash
ros2 launch network_systems main_launch.py
```

This is the best option if multiple modules need to be run at once. Launch
configurations are found under the [config](config/) folder. These
configurations define which modules to enable/disable and what parameters to
use.

### ROS Run

If you just want to run a single module, then this is a direct and easy way
to do it.

For example:

```bash
ros2 run network_systems example --ros-args -p enabled:=true
```

### Binary

Not recommended as you cannot pass ROS parameters, so modules may not work by
default. Binaries for each module found under [projects](projects/) can be
found under
<!-- markdownlint-disable-next-line MD013 -->
`/workspaces/sailbot_workspace/build/network_systems/projects/{module_name}/{module_name}`.

For example:

```bash
/workspaces/sailbot_workspace/build/network_systems/projects/example/example
```

## Testing

Unit tests specific to Network Systems is done using
[GoogleTest](https://github.com/google/googletest). Unit tests are defined per
module.

### Run All Tests

**Option A**: With sailbot_workspace open, invoke the VSCode `test` task.

**Option B**: Under the sailbot_workspace directory, run
`/workspaces/sailbot_workspace/scripts/test.sh`

Both options will run all of UBC Sailbot's tests, including those from other
projects. More often than not, this is unnecessary.

### Run and Debug Specific Tests

This is the preferred way to run and debug tests. When you open a test source
file like
[the example's](projects/can_transceiver/test/test_can_transceiver.cpp), there
will be green arrows next to each `TEST_F` macro. Clicking a double green
arrow runs a test suite, while clicking single green arrow runs one unit test.
Right clicking either arrow will open a prompt with a debug test option. When
running a test via the debug option, we can set breakpoints and step through
our code line by line to resolve issues.

This convenient testing frontend is thank's to the
<!-- markdownlint-disable-next-line MD013 -->
[TestMate extension](https://marketplace.visualstudio.com/items?itemName=matepek.vscode-catch2-test-adapter).

**Warning**: Large failing tests can crash VSCode. If this happens, either
lower the size of the tests (ex. reduce the number of iterations) or
[run the test binary directly](#run-test-binaries).

### Run Test Binaries

Test binaries for each module found under projects can be found under
<!-- markdownlint-disable-next-line MD013 -->
`/workspaces/sailbot_workspace/build/network_systems/projects/{module_name}/test_{module_name}`.

For example:

<!-- markdownlint-disable MD013 -->
```bash
/workspaces/sailbot_workspace/build/network_systems/projects/example/test_example
```
<!-- markdownlint-enable MD013 -->
