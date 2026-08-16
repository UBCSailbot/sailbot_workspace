# Failure-Injection Test Analysis (2026_08_16-08_38_09_934737)

## Test Summary
- **Batch ID**: 2026_08_16-08_38_09_934737
- **Test Plan**: failure_injection.yaml
- **Test Status**: FAILED
- **Test Duration**: 2 hours 20 minutes (8439.566 seconds)
- **Failure Reason**: ROS nodes were missing for 10 seconds: `/navigate_main`
- **Failure Detection**: The test runner detected that `/navigate_main` was missing for 10+ seconds and terminated the test

## Root Cause: Segmentation Fault in navigate_main

### Key Finding
The `/navigate_main` process crashed with a **segmentation fault (SIGSEGV)** exit code `-11` at:
- **Timestamp**: 1786903094.249386530 (approximately 10:58:14 UTC, ~2 hours 20 min into test)
- **Launch log line**: Contains error message:
  ```
  [ERROR] [navigate-2]: process has died [pid 22507, exit code -11, cmd '/workspaces/sailbot_workspace/install/lib/local_pathfinding/navigate ...']
  ```

### Crash Trigger
The last log message before the crash shows:
```
[navigate-2] [INFO] [1786903094.249386530] [navigate_main.local_path:312]: Path is expired
```

**The navigate_main process crashed immediately after detecting an expired local path.**

### What Happened in the Test Plan
At the time of the crash, the test plan had just executed a `mock_global_path` update event:
- **Event Timestamp**: 7800.0 seconds into the 9000-second test
- **Actual crash time**: ~8394 seconds (near end of test)
- Multiple global path updates had occurred throughout the test at: 1500s, 4200s, 6000s, and 7800s

### Consequence Chain
1. navigate_main detected an expired local path and attempted to handle it
2. A segmentation fault occurred during path expiration handling
3. The navigate_main process exited with code -11
4. The test runner continued checking for nodes every 1 second
5. After 10 consecutive seconds without detecting navigate_main, the test runner failed the test
6. Test status: **FAILED** due to missing `/navigate_main` node

## Rosbag Findings
- Total messages recorded: 37,927 across 4 topics
- Topics recorded:
  - `/gps`: 8,432 messages (full duration)
  - `/filtered_wind_sensor`: 8,432 messages (full duration)
  - `/ais_ships`: 16,863 messages (full duration)
  - `/local_path`: 4,200 messages (stopped at ~52 seconds before crash)

- **Last `/local_path` message**: 1786903092.255165342 (timestamp)
- **Last navigate_main message**: 1786903094.249386530 (timestamp)
- **Time gap**: ~2 seconds before the full process crash

## Code Location of Issue
The error originates from `local_path:312` in the navigate_main code. This is likely in the pathfinding module where expired path handling occurs.

## Next Steps for Resolution
1. **Examine pathfinding code** around line 312 where "Path is expired" is checked
2. **Add defensive checks** around path expiration handling to prevent segmentation faults
3. **Add nullptr/boundary checks** in the path data structure operations
4. **Validate** that path updates during the failure injection don't create invalid state
5. **Consider adding error recovery** for expired paths rather than crashing

## Test Completion Status
The test was interrupted by the test runner after detecting missing nodes, not by user interrupt. The final states were:
- Distance covered: 1,010,439 meters
- Target waypoint index: 1 (of 2)
- Remaining route distance: 1,514,068 meters
