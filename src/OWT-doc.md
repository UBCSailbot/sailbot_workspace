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
