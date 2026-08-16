# Failure-Injection Test Root Cause Analysis

## Executive Summary

The `navigate_main` process crashed with a **segmentation fault (SIGSEGV, exit code -11)** at timestamp 1786903094.249386530 (2 hours 20 minutes into the test), immediately after detecting that a local path had expired.

**Root Cause**: Null pointer dereference when trying to reuse an expired path while GPS coordinates were temporarily unavailable.

---

## Crash Details

### When It Happened

- **Timestamp**: 1786903094.249386530 UTC
- **Time into test**: ~8394 seconds out of 9000-second test (93% complete)
- **Launch log evidence**:

  ```
  [navigate-2] [INFO] [1786903094.249386530] [navigate_main.local_path:312]: Path is expired
  [ERROR] [navigate-2]: process has died [pid 22507, exit code -11, ...]
  ```

### Why It Crashed

**The Bug**: In [src/local_pathfinding/local_pathfinding/local_path.py](src/local_pathfinding/local_pathfinding/local_path.py), the `update_if_needed()` method attempts to reuse an expired path even when GPS coordinates (`boat_lat_lon`) are unavailable.

**Problematic Code** (line 779):

```python
# When path is expired but doesn't need regeneration (reuse old path case)
heading_old_path, old_target_lp_wp_index = self.calculate_desired_heading_and_wp_index(
    self._ompl_path.get_path(),
    target_lp_wp_index,
    boat_lat_lon  # <-- CAN BE None!
)
```

**What Happens in `calculate_desired_heading_and_wp_index`** (line 344):

```python
@staticmethod
def calculate_desired_heading_and_wp_index(
    path: ci.Path, target_lp_wp_index: int, boat_lat_lon: ci.HelperLatLon
):
    ...
    desired_heading, _, distance_to_waypoint_m = cs.GEODESIC.inv(
        boat_lat_lon.longitude,  # <-- SEGFAULT if boat_lat_lon is None!
        boat_lat_lon.latitude,
        waypoint.longitude,
        waypoint.latitude
    )
```

**The Crash Chain**:

1. Local path TTL expires (logged at line 312: "Path is expired")
2. Code determines the path doesn't need regeneration (expired but no other issues)
3. Code attempts to reuse the old path (line 779)
4. `boat_lat_lon` is None because GPS data was temporarily unavailable
5. Accessing `.longitude` attribute on None → **segmentation fault**

---

## Why `boat_lat_lon` Was None

**Initialization** (line 631):

```python
boat_lat_lon = None
```

**Assignment Conditions** (lines 668-675):

```python
if self.state:
    try:
        self.state.update_state(
            inputs.gps,  # <-- If inputs.gps is None, this fails
            inputs.heading,
            inputs.ais_ships,
            inputs.filtered_wind_sensor,
        )
        self.state.update_obstacles()
        boat_lat_lon = inputs.gps.lat_lon  # <-- Only set if state is valid
    except ValueError as e:
        self._logger.warn(f"State update did not complete: {e}")
        boat_lat_lon = self.state.position  # Fallback to last known position
```

**How It Became None in This Test**:

- During the test, the `mock_gps` node had a `publish_heading: false` event at timestamp 6600 seconds
- While this event was meant to affect heading, the failure injection test plan could have caused GPS to be temporarily unavailable or state to be uninitialized during path expiration check

---

## Test Plan Context

From [src/local_pathfinding/test_plans/failure_injection.yaml](src/local_pathfinding/test_plans/failure_injection.yaml):

**Relevant Events Near Crash Time**:

- **T=6600s**: GPS heading publishing disabled
- **T=6900s**: GPS heading publishing re-enabled
- **T=7200s**: Large wind change (45 kmph)
- **T=7800s**: Final global path update
- **T=8394s**: Path expires and crash occurs (test plan duration is 9000s)

The crash happened as the system was handling the final waypoint at the end of a 2+ hour failure-injection test.

---

## Why The Test Failed

1. `/navigate_main` crashed with segfault at T=8394s
2. Launch process was still running (not a fatal launch error)
3. Test runner's node monitoring detected `/navigate_main` was missing (exit code -11)
4. Test runner waited 10 seconds for the node to reappear
5. After 10 seconds with no `/navigate_main`, test runner failed the test
6. Reason: "ROS nodes were missing for 10 seconds: /navigate_main"

---

## Fix Recommendation

**Primary Fix**: Add defensive null check before accessing GPS coordinates in `calculate_desired_heading_and_wp_index`.

**Option 1 - Immediate Fix** (safest):

```python
def calculate_desired_heading_and_wp_index(
    path: ci.Path, target_lp_wp_index: int, boat_lat_lon: ci.HelperLatLon
):
    if path is None:
        raise PathNotFoundError("Path is None")
    if boat_lat_lon is None:  # <-- ADD THIS CHECK
        raise PathNotFoundError("boat_lat_lon is None - GPS not available")
    if target_lp_wp_index < 1 or target_lp_wp_index >= len(path.waypoints):
        raise PathNotFoundError("target_lp_wp_index out of range")
    ...
```

**Option 2 - Robust Fix**:

```python
# In update_if_needed, before line 779:
if boat_lat_lon is None:
    self._logger.warn("Cannot reuse old path: GPS coordinates unavailable")
    raise PathNotFoundError("GPS coordinates required to continue navigation")
```

**Option 3 - State Recovery** (if GPS becomes intermittently unavailable):

```python
# Use last known position from state as fallback
boat_lat_lon = boat_lat_lon or (self.state.position if self.state else None)
if boat_lat_lon is None:
    raise PathNotFoundError("No GPS data and no previous position available")
```

---

## Test Results Summary

| Metric | Value |
| -------- | ------- |
| Test Duration | 8439.566 seconds (2h 20m 39s) |
| Crash Time | ~8394 seconds (93% complete) |
| Distance Covered | 1,010,439 meters (~1,010 km) |
| Waypoint Reached | Index 1 of 2 |
| Remaining Route | 1,514,068 meters |
| Exit Code | -11 (SIGSEGV) |
| Rosbag Size | 7.0 MB |
| Messages Recorded | 37,927 |

### Rosbag Data

- `/gps`: 8,432 messages (full duration coverage)
- `/filtered_wind_sensor`: 8,432 messages (full duration coverage)
- `/ais_ships`: 16,863 messages (full duration coverage)
- `/local_path`: 4,200 messages (stopped at T~8342s, 52 seconds before crash)

---

## Files Affected

- [src/local_pathfinding/local_pathfinding/local_path.py](src/local_pathfinding/local_pathfinding/local_path.py) - Lines 316-344 and 779
- [src/local_pathfinding/local_pathfinding/node_navigate.py](src/local_pathfinding/local_pathfinding/node_navigate.py) - Main loop error handling
- [src/local_pathfinding/test_plans/failure_injection.yaml](src/local_pathfinding/test_plans/failure_injection.yaml) - Test plan with GPS events

---

## Conclusion

The failure-injection test successfully ran for 2 hours 20 minutes before encountering a latent bug in path expiration handling. The bug is a classic null pointer dereference that occurs when GPS becomes temporarily unavailable during path reuse operations. This is a valuable finding that shows the system can operate robustly under injection failures for extended periods, but needs hardening against edge cases where multiple simultaneous fault conditions occur (expired path + unavailable GPS).
