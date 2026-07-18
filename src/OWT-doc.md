<!-- markdownlint-disable -->
# On-Water Testing Objectives

This file lists the on-water tests we want to run and the observations needed
to mark each test as pass or fail.

## CTRL Tests

### Objective

Verify that the wingsail controller accepts a desired-heading test input,
commands the actuator, and produces the matching real-world trim-tab response.

The controller README describes the manual control test script:

```bash
./scripts/test_ctrl_manual.sh <speed_kmph> <wind_direction_deg> <desired_heading_deg>
```

The script launches the controller, publishes `/filtered_wind_sensor`, and
publishes `/desired_heading` with `sail: true`.

Current controller behavior to verify:

- Desired heading is included in the input set and confirms the controller is
  receiving the `/desired_heading` command.
- Exact computed angle values are covered by the controller unit tests. This
  on-water test only verifies the physical trim-tab output.
- Positive and negative command cases should move the trim tab to opposite
  sides.
- Zero-command cases should leave the trim tab centered or return it to center.
- The actuator should move smoothly, settle, and hold position.

### Pass/Fail Criteria

For each test, record the input set, the physical trim-tab observation, and
pass/fail.

Pass if:

- The physical trim tab moves to the expected side for nonzero commands.
- Positive and negative command cases move to opposite sides.
- For zero-command cases, the trim tab remains centered or returns to center.
- The actuator settles without oscillating, binding, or drifting.

Fail if:

- The trim tab moves to the wrong side or does not move for a nonzero command.
- The trim tab moves when the expected physical output is centered.
- The actuator keeps moving, oscillates, binds, or fails to hold position.

### Control Test Matrix

| ID | Input set | Expected physical output | Real-world observation | Pass/Fail |
| --- | --- | --- | --- | --- |
| CTRL-01 | `speed=10 kmph`, `wind_direction=45 deg`, `desired_heading=90 deg` | Trim tab moves to the positive-command side and settles. |  |  |
| CTRL-02 | `speed=10 kmph`, `wind_direction=-45 deg`, `desired_heading=90 deg` | Trim tab moves to the negative-command side and settles. |  |  |
| CTRL-03 | `speed=10 kmph`, `wind_direction=45 deg`, `desired_heading=-90 deg` | Trim tab moves to the same side as CTRL-01 and settles. |  |  |
| CTRL-04 | `speed=0 kmph`, `wind_direction=45 deg`, `desired_heading=90 deg` | Trim tab remains centered or returns to center. |  |  |
| CTRL-05 | `speed=20 kmph`, `wind_direction=180 deg`, `desired_heading=-90 deg` | Trim tab moves to the positive-command side and settles. |  |  |
| CTRL-06 | `speed=30 kmph`, `wind_direction=-120 deg`, `desired_heading=0 deg` | Trim tab moves to the negative-command side and settles. |  |  |
| CTRL-07 | `speed=46.3 kmph`, `wind_direction=45 deg`, `desired_heading=0 deg` | Trim tab remains centered or returns to center. |  |  |

TODO: Run this for next OWT, we had issues with getting this to actuate.
Make sure that the input/outputs match the trim tab actuation limits (eg. July 11 it was -12 to 3 degrees) 

Run each row from the workspace root. Example:

```bash
./scripts/test_ctrl_manual.sh 10 45 90
```

Then observe the physical trim tab and fill in the `Real-world observation` and
`Pass/Fail` columns.


### NET
We want to run the entire SOFT system and NET should properly receive, parse, and publish the sensor data. This will be
a passive test where we don't touch the boat much.

If possible, we would like to send a new set of global waypoints over the satellite.
#### Stages for sending global waypoints:
| len(waypoints) | Pass / Fail |
| 1 | |
| 2 | |
| 3 | |
| 5 | |

### PATH
We will have to comprehensively test PATH. We will do this in stages.
**Ensure that we start the test from the pier (check the image attached below)**.
#### Stage 1
Run PATH without any mock ais being published and two basic global waypoints. Let pathfinding run and Sailbot reach
one of the waypoints.

#### Stage 2
Run PATH one mock AIS right in front of the boat being published and two basic global waypoints. Let pathfinding run and Sailbot reach one of the waypoints and clear the AIS.

#### Stage 3
Run PATH with multiple mock AIS around the boat and let it navigate. Verify the tacks and the general behavior of PATH.

##### 3a
Let this configuration run of 10 minutes and verify that the PATH changes. One can also verify that the PATH changes
because of wind shift or a collision zone appearing in the picture.

##### 3b (Not for OWT on 11 July)
Publish a new collision zone at a relative time stamp to verify that the boat changes paths.

#### Stage 4: Chase-boat integration cases

Run these with normal GPS, wind, AIS, and global path inputs unless the setup says otherwise.
Prioritize these before sensor-failure tests because they verify the behavior we expect while we are
following Sailbot from the chase boat.

Useful observation commands:

```bash
ros2 topic echo /desired_heading
ros2 topic echo /local_path
```

| Priority | ID | Setup | Expected behavior | Real-world observation | Pass/Fail |
| --- | --- | --- | --- | --- | --- |
| P0 | PATH-CB-01 | While PATH is actively sailing, trigger the normal manual takeover or kill behavior. | Sailbot stops autonomous control quickly and predictably. Record the time from command to safe state. |  |  |
| P0 | PATH-CB-02 | Use 3-5 global waypoints and let Sailbot sail through them. | PATH advances through the global waypoints in order, keeps `/desired_heading.sail=true` when a local path is available, and updates the local path at each new global waypoint. |  |  |
| P0 | PATH-CB-03 | Send a new global path while Sailbot is already sailing. | PATH accepts the new route, replans toward the new target, and does not make an unsafe heading jump. |  |  |
| P1 | PATH-CB-04 | Publish a valid global path, let PATH accept it, then stop sending new global path messages. | PATH continues using the accepted route and does not reset waypoint progress. |  |  |
| P1 | PATH-CB-05 | Use exactly two global waypoints and let Sailbot reach the final global waypoint, which is index `0` in PATH's reverse-ordered global path. | PATH switches back to the second-to-last global waypoint at index `1` instead of disabling sail. If Sailbot reaches index `1` again, PATH switches back to index `0`. |  |  |
| P1 | PATH-CB-06 | Let Sailbot free-run for 10-15 minutes after a valid local path is generated. | PATH remains stable. Any local path changes should be explained by waypoint progress, wind shift, collision zone changes, or path timeout. |  |  |
| P1 | PATH-CB-07 | Restart PATH while Sailbot is on the water and a valid route exists. | PATH resumes from the available route and current GPS position without sending Sailbot backward along the route. |  |  |
| P2 | PATH-CB-08 | Record the whole PATH run. | Rosbag or logs include `/gps`, `/desired_heading`, `/local_path`, `/filtered_wind_sensor`, `/ais_ships`, and `/global_path`. |  |  |
| P2 | PATH-CB-09 | Watch the visualizer or base-station view during the run. | Observers can see current GPS, global path, local path, obstacles, desired heading, and sail state clearly enough to make chase-boat decisions. |  |  |
| P2 | PATH-CB-10 | Startup with no `/global_path` from NET or mock global path. | If no persisted fallback path is available, PATH logs missing `global_path`, publishes `/desired_heading.sail=false`, and waits for a global path. |  |  |
| P2 | PATH-CB-11 | Publish an empty `/global_path` message. | If an active path already exists, PATH keeps the current in-memory path. If no active path exists, PATH tries persisted fallback and otherwise keeps sail disabled. |  |  |

#### Stage 5: Sensor failure and obstacle cases

AIS is the only controllable sensor failure we have for this OWT right now. GPS and wind
failure rows should stay in the plan, but they are only possible after NET implements a way
to switch off sensor publishing or publish sensor failure states.

| Priority | ID | Availability | Setup / fault injected | Expected behavior | Real-world observation | Pass/Fail |
| --- | --- | --- | --- | --- | --- | --- |
| P1 | PATH-AIS-01 | Available now | Startup with `/ais_ships` not publishing. | PATH logs missing `ais_ships`, publishes `/desired_heading.sail=false`, and waits for AIS input before initial planning. |  |  |
| P1 | PATH-AIS-02 | Available now | Let PATH publish a valid heading, then stop `/ais_ships`. | PATH should keep following the current local path if no replan is required. If a replan is required and AIS is still missing, record whether it fails safe. |  |  |
| P1 | PATH-AIS-03 | Available now | Start nominal, then publish one mock AIS obstacle in front of Sailbot. | PATH changes the local path to clear the obstacle and keeps `/desired_heading.sail=true` if a safe local path exists. |  |  |
| P2 | PATH-AIS-04 | Available now | Use mock AIS geometry that blocks every local route to the target global waypoint. | PATH publishes `/desired_heading.sail=false`, publishes an empty local path, logs that it cannot generate a local path, and does not silently advance the global waypoint. |  |  |
| P0 | PATH-SF-01 | Requires NET sensor failure support | Let PATH publish a valid heading, then stop `/gps` publishing for more than 120 seconds. | `/desired_heading.sail` becomes `false`, heading becomes `0.0`, and PATH logs that GPS has not been received for more than 120 seconds. |  |  |
| P0 | PATH-SF-02 | Requires NET sensor failure support | Startup with `/gps` not publishing or publishing a failure state. | PATH logs missing or failed `gps`, publishes `/desired_heading.sail=false`, and does not publish a stale local path as valid. |  |  |
| P0 | PATH-SF-03 | Requires NET sensor failure support | Startup with `/filtered_wind_sensor` not publishing or publishing a failure state. | PATH logs missing or failed `filtered_wind_sensor`, publishes `/desired_heading.sail=false`, and waits for wind data before planning. |  |  |
| P1 | PATH-SF-04 | Requires NET sensor failure support | Let PATH publish a valid heading, then stop `/filtered_wind_sensor` or publish a wind sensor failure state. | PATH should fail safe if a replan is required: `/desired_heading.sail=false` and local path empty. If the current path is reused temporarily, record how long and whether it later fails safe. |  |  |


## 11th July On-Water Test Plan


Complete the dry-land testing.

- [ ] CTRL-01 — speed=10 kmph, wind_direction=45 deg, desired_heading=90 deg → Trim tab moves to the positive-command side and settles.
- [ ] CTRL-02 — speed=10 kmph, wind_direction=-45 deg, desired_heading=90 deg → Trim tab moves to the negative-command side and settles.
- [ ] CTRL-03 — speed=10 kmph, wind_direction=45 deg, desired_heading=-90 deg → Trim tab moves to the same side as CTRL-01 and settles.
- [ ] CTRL-04 — speed=0 kmph, wind_direction=45 deg, desired_heading=90 deg → Trim tab remains centered or returns to center.
- [ ] CTRL-05 — speed=20 kmph, wind_direction=180 deg, desired_heading=-90 deg → Trim tab moves to the positive-command side and settles.
- [ ] CTRL-06 — speed=30 kmph, wind_direction=-120 deg, desired_heading=0 deg → Trim tab moves to the negative-command side and settles.
- [ ] CTRL-07 — speed=46.3 kmph, wind_direction=45 deg, desired_heading=0 deg → Trim tab remains centered or returns to center.

#### Running the CTRL tests with the script

```bash
# Valid manual controller tests
./scripts/test_ctrl_manual.sh 10 45 90      # CTRL-01
./scripts/test_ctrl_manual.sh 10 -45 90     # CTRL-02
./scripts/test_ctrl_manual.sh 10 45 -90     # CTRL-03
./scripts/test_ctrl_manual.sh 0 45 90       # CTRL-04
./scripts/test_ctrl_manual.sh 20 180 -90    # CTRL-05
./scripts/test_ctrl_manual.sh 30 -120 0     # CTRL-06
./scripts/test_ctrl_manual.sh 46.3 45 0     # CTRL-07

# Invalid input validation (script must reject these with an error)
./scripts/test_ctrl_manual.sh 10 181 90     # wind_direction out of [-180, 180]
./scripts/test_ctrl_manual.sh 10 45 -181    # desired_heading out of [-180, 180]
./scripts/test_ctrl_manual.sh -1 45 90      # speed must be >= 0
./scripts/test_ctrl_manual.sh fast 45 90    # non-numeric argument
```

#### Running the CTRL tests manually with ros2 (if the script is unavailable)

Each script invocation is equivalent to the following. Use one terminal per
command and source the workspace in each terminal first:

```bash
source install/local_setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}" && mkdir -p "$ROS_LOG_DIR"
```

Terminal 1 — launch the wingsail controller (leave running for all tests):

```bash
ros2 launch controller main_launch.py
```

Terminal 2 — publish the apparent wind (substitute `speed` and `direction` per test):

`direction` is flow-toward in the boat frame: 0° means airflow travels from
bow toward stern, and values increase clockwise.

```bash
ros2 topic pub /filtered_wind_sensor custom_interfaces/msg/WindSensor \
    "{speed: {speed: 10.0}, direction: 45}" -r 1
```

Terminal 3 — publish the desired heading (substitute `heading` per test):

```bash
ros2 topic pub /desired_heading custom_interfaces/msg/DesiredHeading \
    "{heading: {heading: 90.0}, steering: 0, sail: true}" -r 1
```

Terminal 4 (optional) — observe the commanded trim-tab angle alongside the
physical trim tab:

```bash
ros2 topic echo /sail_cmd
```

Per-test publisher values (Ctrl-C the two `ros2 topic pub` commands between
tests and rerun with the next row's values; the controller can stay up):

| ID | Terminal 2: `/filtered_wind_sensor` | Terminal 3: `/desired_heading` |
| --- | --- | --- |
| CTRL-01 | `"{speed: {speed: 10.0}, direction: 45}"` | `"{heading: {heading: 90.0}, steering: 0, sail: true}"` |
| CTRL-02 | `"{speed: {speed: 10.0}, direction: -45}"` | `"{heading: {heading: 90.0}, steering: 0, sail: true}"` |
| CTRL-03 | `"{speed: {speed: 10.0}, direction: 45}"` | `"{heading: {heading: -90.0}, steering: 0, sail: true}"` |
| CTRL-04 | `"{speed: {speed: 0.0}, direction: 45}"` | `"{heading: {heading: 90.0}, steering: 0, sail: true}"` |
| CTRL-05 | `"{speed: {speed: 20.0}, direction: 180}"` | `"{heading: {heading: -90.0}, steering: 0, sail: true}"` |
| CTRL-06 | `"{speed: {speed: 30.0}, direction: -120}"` | `"{heading: {heading: 0.0}, steering: 0, sail: true}"` |
| CTRL-07 | `"{speed: {speed: 46.3}, direction: 45}"` | `"{heading: {heading: 0.0}, steering: 0, sail: true}"` |

Notes:

- `speed.speed` and `heading.heading` are `float32`; `direction` is `int16`
  (whole degrees only, in `(-180, 180]`).
- The manual path has no input validation — the script's invalid-input cases
  (`181`, `-181`, `-1`, `fast`) only apply when testing the script itself.
- `sail: true` is required; the controller treats `sail: false` as a
  pathfinding failure and the heading is ignored.

### Stage 1 testing for PATH

Run PATH with no mock AIS and exactly two basic global waypoints, and let
Sailbot reach one of them. **Start the test from the pier.**

#### Setup

Launch the full system in production mode from the workspace root. The
on-water config defaults `on_water_mock_ais` to `True`, so it must be
overridden off for Stage 1:

```bash
ros2 launch src/global_launch/main_launch.py \
    mode:=production \
    config:=on_water_globals.yaml \
    on_water_mock_ais:=false \
    record:=true
```

- `record:=true` starts a rosbag (saved under
  `notebooks/local_pathfinding/session_recordings` by default) — this covers
  PATH-CB-08.
- The on-water config enables `visualizer_mode`, which launches the
  `navigate_observer` node for PATH-CB-09.
- Real AIS must still be publishing `/ais_ships` (via NET). Without it, PATH
  waits with `/desired_heading.sail=false` before initial planning
  (see PATH-AIS-01).

Before starting, check the persisted global path in
`src/local_pathfinding/local_pathfinding/global_path_storage/main_global_path.csv`.
PATH auto-loads this on startup, so a stale path from a previous session will
be used until a new `/global_path` arrives. Delete it or confirm it matches
the intended Stage 1 waypoints.

#### Sending the two global waypoints

Preferred: have NET send the two waypoints (this doubles as the NET
`len(waypoints)=2` row). Fallback: publish `/global_path` manually.

**The global path is reverse-ordered: index 0 is the final destination and
the last element is the first target.** For two waypoints, list the final
destination first. Example using Jericho-area coordinates (boat sails out to
the bay waypoint, then back toward the launch waypoint):

```bash
ros2 topic pub /global_path custom_interfaces/msg/Path \
    "{waypoints: [{latitude: 49.278570, longitude: -123.197598}, {latitude: 49.283240, longitude: -123.195196}]}" --once
```

A global waypoint counts as reached when Sailbot is within 300 m of it
(`GLOBAL_WAYPOINT_REACHED_THRESH_M` in `node_navigate.py`).

#### Observation

```bash
ros2 topic echo /desired_heading   # expect sail: true once a local path exists
ros2 topic echo /local_path
ros2 topic echo /gps
```

Also watch the visualizer / base-station view for GPS, global path, local
path, and desired heading.

#### Stage 1 checklist

| ID | What to do / expect | Real-world observation | Pass/Fail |
| --- | --- | --- | --- |
| PATH-CB-05 | Use exactly two global waypoints. When Sailbot reaches the final waypoint (index 0), PATH switches back to the waypoint at index 1 instead of disabling sail; reaching index 1 again switches back to index 0. |  | [ ] Pass / [ ] Fail |
| PATH-CB-06 | After a valid local path is generated, let Sailbot free-run 10–15 minutes (run this when it occurs naturally). PATH stays stable; any local path change is explained by waypoint progress, wind shift, or path timeout. |  | [ ] Pass / [ ] Fail |
| PATH-CB-08 | Recording — already happening via `record:=true`. At the end, confirm the bag includes `/gps`, `/desired_heading`, `/local_path`, `/filtered_wind_sensor`, `/ais_ships`, `/global_path`. |  | [ ] Pass / [ ] Fail |
| PATH-CB-09 | Watch the visualizer or base-station view throughout. Observers can see GPS, global path, local path, obstacles, desired heading, and sail state clearly enough for chase-boat decisions. |  | [ ] Pass / [ ] Fail |
