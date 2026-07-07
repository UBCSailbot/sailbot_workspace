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


