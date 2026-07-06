# Local Pathfinding

UBC Sailbot's local pathfinding ROS package

## Run

Using main launch file: `ros2 launch local_pathfinding main_launch.py`

## Test Plans

Test plans live in `test_plans/` and are documented in the
[Test Plans Confluence page](https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/2996568069/Test+Plans).
Use `test_plan:=<plan>.yaml` with the local pathfinding launch file to run one
plan manually:

```bash
ros2 launch local_pathfinding main_launch.py mode:=development test_plan:=basic.yaml
```

To run test plans sequentially, build and source the workspace, then use the
installed `run_test_plans` console script:

```bash
./scripts/build.sh -p local_pathfinding
source install/local_setup.bash
ros2 run local_pathfinding run_test_plans --list
ros2 run local_pathfinding run_test_plans --tests basic.yaml --num_tests 1
```

`--tests` accepts test plan names or list numbers from `--list`. If
`--num_tests` is larger than the explicitly selected plans, the runner fills the
remaining slots randomly; use `--seed` for repeatable random selection. Results
and rosbag recordings are written under
`notebooks/local_pathfinding/session_recordings/test_plans_results/` by default.

## Wind Tracking

Local pathfinding keeps a rolling true-wind history in `WindTracker` so wind
history survives `LocalPathState` replacement during path regeneration. Once the
history reaches `WIND_HISTORY_LEN` readings, `WindTracker.tw_avg` is used as the
smoothed true wind estimate.

Each generated local path stores `path_generated_wind`, the true wind value
used by OMPL for that path. This is the rolling average when available; before
the rolling average is populated, it falls back to the current filtered apparent
wind reading converted to true wind. Wind-based path switching compares the current
rolling average against `path_generated_wind`, so a path is regenerated only when
the smoothed wind has drifted significantly from the wind used to generate the
current path.

### Launch Parameters

Launch arguments are added to the run command in the format `<name>:=<value>`.

<!-- markdownlint-disable MD013 -->
| name        | description   | value                                                 |
| ----------- | ------------- | ----------------------------------------------------- |
| `log_level` | Logging level (default: `{INFO}`) | A [severity level](https://docs.ros.org/en/humble/Concepts/About-Logging.html#severity-level) (case insensitive) `{DEBUG, INFO, WARN, ERROR, and FATAL}` |
|`mode` | Mode (default: `development`) | `{development, production}`|
|`test_plan` | Test Plan is not required when `mode=production` | any test definition yaml file in `local_pathfinding/test_plans`. (eg. `test_plan:=basic.yaml`) |
|`use_gps_noise`|Enable GPS noise (default:`true`)|`{true, false}`|
|`use_ocean_drift`|Enable ocean current drift (default:`true`)|`{true, false}`|
|`use_drift_randomization`|Enable ocean current noise (default:`true`)|`{true, false}`|
|`ocean_drift_speed_kmph`|Speed of the ocean drift (km/h)(default:`0.5`)|Any float|
|`ocean_drift_dir_deg`|Direction of the ocean drift in degrees (0 = North, 90 = East) (default:`45.0`)|Any float between `(-180, 180]`|
|`ocean_drift_accel_kmph2`|Acceleration of ocean drift speed (km/h^2)(default:`0.0`)|Any float|
<!-- markdownlint-enable MD013 -->

## Some other important commands

<!-- markdownlint-disable MD013 -->
- `ros2 topic list`: Lists all the topics ([ROS 2 topic list documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-list))
- `ros2 node list`: Lists all the nodes that are running
- `ros2 topic echo <topic name>`, e.g.: `ros2 topic echo /filtered_wind_sensor`: ([ROS2 topic echo documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-echo:~:text=to%20avoid%20confusion.-,4%20ros2%20topic%20echo,-%EF%83%81))
- You can also do `ros2 topic echo <topic name> --field <field name>` to isolate
  a field in a topic (eg. `speed` in `\filtered_wind_sensor`)
<!-- markdownlint-enable MD013 -->
- `ros2 <command> -h` can be used to get more info about any command.

## Example Launch Command
`ros2 launch local_pathfinding main_launch.py mode:=development use_gps_noise:=true use_ocean_drift:=true use_drift_randomization:=true ocean_drift_speed_kmph:=0.5 ocean_drift_dir_deg:=45.0 ocean_drift_accel_kmph2:=0.0`
