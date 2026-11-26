
## Run started 2026-08-16 03:26:07

- log: `/workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recordings/test_plans_results/2026_08_16-03_25_39_447711/overnight_dynamic_stress/launch.log`
- interval: 900s, stall limit: 1800s, model: haiku

### 2026-08-16 05:56:19

- **Persistent OMPL planner failure** (every ~2 sec, starting 1786884938.852): RRTstar reports "no valid initial states" on both path-generation attempts, causing sail to be disabled repeatedly. Boat stuck at waypoint index 22 (lat 49.344, lon -123.276).

### 2026-08-16 06:13:10

- **Repeated planner failure** (1786885950.854–1786885979): RRTstar consistently reports "no valid initial states" across both planning attempts, failing every cycle; sails disabled as fallback. Boat stuck at waypoint index 22 (49.344°N, 123.276°W) while actual position is ~49.336°N, 123.257°W.

### 2026-08-16 06:30:04

**Repeated OMPL path planning failures.** RRTstar planner consistently reports "Skipping invalid start state" and "no valid initial states" (~every 4–6 seconds throughout the log: 1786886960, 1786886964, 1786886966, 1786886968, 1786886970, 1786886972, 1786886974, 1786886976, 1786886978, 1786886980, 1786886982, 1786886984, 1786886986, 1786886988). Both planning attempts fail each time, sail is disabled, and the boat holds heading 0.0 while aiming for waypoint index 22 at (49.344°N, 123.276°W). The boat is moving but path planning is broken.

### 2026-08-16 06:46:56

**OMPL local pathfinding systematically failing** — RRTstar consistently reports "no valid initial states" since [1786887974.85], repeating every ~2s with both planning attempts failing per cycle. Each failure disables the sail (desired_heading: 0.0, sail=False), yet the boat continues moving with GPS updates normal. This suggests the planner cannot validate the current state despite ~30s of operation.

### 2026-08-16 07:03:47

**Persistent RRTstar planner failure** — Every ~2 seconds from 07:03:06 onward, path planning attempts fail with "invalid start state" and "no valid initial states." Boat remains stuck at waypoint 22 (49.344°N, 123.276°W), unable to generate local paths; sail disabled repeatedly.

### 2026-08-16 07:20:38

- **[1786889387] navigate-2 crashed with segmentation fault (exit code -11)** — process died mid-simulation before graceful shutdown
- **Repeated "Path intersects with collision zone" (every 2–3s throughout)** — planner repeatedly detecting path collisions, suggesting it may be cycling through similar avoidance scenarios rather than finding clear paths

## 2026-08-16 07:20:38 — RUN ENDED

```json
{
  "command": [
    "ros2",
    "launch",
    "global_launch",
    "main_launch.py",
    "mode:=development",
    "config:=globals.yaml",
    "log_level:=info",
    "test_plan:=overnight_dynamic_stress.yaml",
    "record:=true",
    "save_path:=notebooks/local_pathfinding/session_recordings/test_plans_results/2026_08_16-03_25_39_447711/overnight_dynamic_stress"
  ],
  "duration_sec": 12133.593,
  "ended_at": "2026-08-16T07:10:20",
  "expected_node_names": [
    "/can_transceiver_node",
    "/local_transceiver_node",
    "/mock_ais",
    "/mock_global_path",
    "/mock_gps",
    "/mock_wind_sensor",
    "/navigate_main",
    "/navigate_observer",
    "/rosbag2_recorder",
    "/wingsail_ctrl_node"
  ],
  "last_distance_m": 4217.180234218266,
  "last_target_waypoint_index": 22,
  "missing_node_names": [
    "/navigate_main"
  ],
  "reason": "ROS nodes were missing for 10 seconds: /navigate_main",
  "remaining_local_waypoints": 2,
  "remaining_route_distance_m": 113466.94383448745,
  "result_path": "/workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recordings/test_plans_results/2026_08_16-03_25_39_447711/overnight_dynamic_stress/result.json",
  "return_code": 0,
  "ros_launch_log_path": "/workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recordings/test_plans_results/2026_08_16-03_25_39_447711/overnight_dynamic_stress/launch.log",
  "ros_launch_log_stored": true,
  "save_path": "notebooks/local_pathfinding/session_recordings/test_plans_results/2026_08_16-03_25_39_447711/overnight_dynamic_stress",
  "status": "FAILED",
  "test": "overnight_dynamic_stress.yaml",
  "test_plan_start_time": "2026-08-16T03:25:39"
}
```

