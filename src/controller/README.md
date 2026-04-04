# Controller

UBC Sailbot's controller for the **Polaris** project. This ROS 2 package computes and publishes trim tab commands for the wingsail based on wind sensor data and desired heading. It is intended for handoff to future engineering students; this README explains the source code, configuration, and how to run and test the package.

---

## Overview

The controller is part of the UBC Sailbot design club's Polaris autonomous sailboat software. It:

- Subscribes to **filtered wind** (speed and direction), **GPS**, and **desired heading** (from pathfinding/navigation).
- Converts apparent wind speed and direction into a **trim tab angle** for the wingsail using a Reynolds-number–based lookup table and scaling logic.
- Publishes **sail commands** (trim tab angle in degrees) at a configurable period.

The wingsail controller’s goal is to keep the wingsail at a desired angle of attack while favoring speed by maximizing lift-to-drag, and to reduce or zero out trim when wind is high or when the system should not sail (e.g. no path found).

---

## Specifications

- **Wind → trim tab:** Takes apparent wind speed and direction and converts them to trim tab angle (degrees) using:
    - Reynolds number from wind speed and sail chord.
    - A lookup table (Reynolds number → angle of attack) with linear interpolation.
    - Sign of the trim tab angle follows apparent wind direction (convention: 0° bow-to-stern, angle increasing clockwise; trim tab range typically ±40°).

- **Wind scaling (mast protection):** If apparent wind speed is between a **lower** and **upper** threshold (parameters in `globals.yaml`), the trim tab is scaled down linearly (1 at lower threshold, 0 at upper). Above the upper threshold, trim tab is forced to 0 to avoid damaging the mast in high wind.

- **Stop sailing when no path:** When pathfinding cannot find a path, the navigation stack sets the desired heading’s `sail` flag to `false`. The controller subscribes to `desired_heading` and, when `sail` is false, publishes trim tab angle 0 and does not drive the boat.

- **Minimum wind speed threshold:** A minimum wind speed threshold for controller activation is planned but **not yet implemented** (to be added).

- **Design reference:** Preliminary design documents: [Polaris design (Confluence)](https://ubcsailbot.atlassian.net/wiki/x/fABxag).

---

## File structure

```
src/controller/
├── controller/                 # Python package (no __pycache__)
│   ├── common/                 # Shared utilities and constants
│   │   ├── constants.py
│   │   ├── lut.py
│   │   └── types.py
│   └── wingsail/
│       ├── controllers.py
│       └── wingsail_ctrl_node.py
├── launch/
│   └── main_launch.py
├── tests/
│   └── unit/
│       └── wingsail/
│           ├── test_controllers.py
│           └── common/
│               └── test_lut.py
├── resource/
│   └── controller
├── package.xml
├── setup.py
├── setup.cfg
├── LICENSE
└── .gitignore
```

(`__pycache__` and other build/install artifacts are ignored.)

---

## Explanation of each file

| File | Purpose |
|------|--------|
| **controller/common/constants.py** | Physical constants: chord width of main sail, kinematic viscosity, and the Reynolds number → angle-of-attack lookup table (used for trim tab computation). |
| **controller/common/lut.py** | Look-up table (LUT) class: loads (x, y) data and performs linear or spline interpolation. Used to map Reynolds number to desired angle of attack. |
| **controller/common/types.py** | Type aliases used in the package (e.g. `Scalar`, `ScalarOrArray`) for type hinting. |
| **controller/wingsail/controllers.py** | `WingsailController`: computes Reynolds number from apparent wind speed, then trim tab angle from Reynolds number and apparent wind direction using the LUT. No ROS dependency. |
| **controller/wingsail/wingsail_ctrl_node.py** | ROS 2 node `wingsail_ctrl_node`: declares parameters, subscribes to `filtered_wind_sensor`, `gps`, and `desired_heading`, applies wind-threshold scaling and `sail` flag, and publishes `SailCmd` (trim tab angle) on a timer. |
| **launch/main_launch.py** | Launch file: loads global launch arguments (including `config` and `log_level` from `global_launch`), then starts the wingsail controller node with the given config. |
| **tests/unit/wingsail/test_controllers.py** | Unit tests for `WingsailController`: Reynolds number, trim tab angle from LUT, and full `get_trim_tab_angle`. |
| **tests/unit/wingsail/common/test_lut.py** | Unit tests for `LUT`: construction, invalid table/interpolation method, linear/spline interpolation and extrapolation. |
| **setup.py** | Package metadata, dependencies (e.g. `numpy`, `scipy`), and entry point for the `wingsail_ctrl_node` executable. |

---

## Configuration parameters (globals.yaml / global_launch)

The controller node is launched with a config file path that defaults to the **global_launch** config. That file is:

`src/global_launch/config/globals.yaml`

The **controller**-relevant parameters are under the `wingsail_ctrl_node` namespace. The launch file passes the path via the `config` launch argument (see `global_launch`’s `GLOBAL_LAUNCH_ARGUMENTS`).

| Parameter | Description |
|-----------|-------------|
| **pub_period_sec** (global) | Period (seconds) at which the wingsail node publishes sail commands. |
| **reynolds_number** | Array of Reynolds numbers for the LUT (x-axis). In the node this is overridden by a hardcoded table in `constants.py`; the param is declared for future use. |
| **angle_of_attack** | Array of angles of attack (degrees) corresponding to `reynolds_number` (y-axis). Same note as above. |
| **apparent_wind_lower_threshold_kmph** | Lower wind speed threshold (e.g. m/s). Below this, scaling factor is 1. Between this and the upper threshold, trim tab is scaled down. |
| **apparent_wind_upper_threshold_kmph** | Upper wind speed threshold. At or above this, scaling factor is 0 (trim tab forced to 0 for mast protection). |

Full parameter descriptions and conventions are in:
`src/global_launch/config/README.md`.

---

## Publishers and subscriptions

| Type | Topic | Message type | Description |
|------|--------|--------------|-------------|
| **Subscription** | `filtered_wind_sensor` | `WindSensor` | Filtered apparent wind speed and direction; used for Reynolds number and trim tab angle. |
| **Subscription** | `gps` | `GPS` | GPS data; stored for future use (e.g. minimum wind threshold or logging). |
| **Subscription** | `desired_heading` | `DesiredHeading` | Desired heading and **sail** flag; when `sail` is false (e.g. no path), controller publishes trim tab 0. |
| **Publisher** | `sail_cmd` | `SailCmd` | Trim tab angle in degrees; published at `pub_period_sec`. |

All use QoS profile 1 (reliable, keep last 1).

---

## Testing

- **Dry land testing:** [Dry land testing (Confluence)](https://ubcsailbot.atlassian.net/wiki/x/EYCRq)
- **In water testing:** [In water testing (Confluence)](https://ubcsailbot.atlassian.net/wiki/x/LwA8qg)

---

## How to launch the package

From the workspace (after sourcing the overlay and building):

```bash
ros2 launch controller main_launch.py
```

Optional arguments (from global_launch) include:

- **config** – Path to the ROS parameter YAML (default: `global_launch`’s `globals.yaml`).
- **log_level** – Logger level: `debug`, `info`, `warn`, `error`, `fatal` (default: `info`).

Example with custom config and debug logging:

```bash
ros2 launch controller main_launch.py config:=/path/to/globals.yaml log_level:=debug
```

To see all options:

```bash
ros2 launch controller main_launch.py -s
```

---

## How to run unit tests

From the **controller** package directory (or workspace root with the package in the path):

```bash
pytest src/controller/tests/unit -v
```

Or, from the workspace root, using the workspace test flow:

- Run the **test** task in the Sailbot Workspace (e.g. via VS Code Command Palette).
- Or use the workspace test script if available (e.g. `scripts/test.sh`), which runs all packages’ tests including the controller.

Unit tests live under `tests/unit/wingsail/` (controllers and LUT).

---

## Integration tests

**TBD.** Integration tests for the controller have not been implemented yet. When they are, this section will be updated with how to run them (e.g. via the `integration_tests` package and testplans).

---

## Remaining work

Summary of remaining tests and implementation tasks:

1. **Implement min wind speed logic** — Add a minimum apparent wind speed threshold so the controller only activates (publishes non-zero trim tab) when wind is above this value; parameterize via `globals.yaml` and use it in the wingsail node before computing/publishing trim tab.

2. **Implement integration tests** — Add controller integration tests (e.g. under the `integration_tests` package with a dedicated testplan) to verify the node with other components (wind, pathfinding, sail actuation) in a single run.

3. **In water testing** — Execute and document in-water validation of the controller using the procedures in [In water testing (Confluence)](https://ubcsailbot.atlassian.net/wiki/x/LwA8qg).
