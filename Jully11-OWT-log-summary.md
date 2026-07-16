<!-- markdownlint-disable -->

## Overall

The log covers **2026-07-08 12:12:33 to 13:02:32**, about **49 minutes 59 seconds**.

The system initially navigated successfully, but the local planner failed permanently at **12:41:14**. From then until the log ended, navigation repeatedly disabled sailing because OMPL considered the boat’s start state invalid.

## Failures

| Timestamp                          | Failure                                                                                                                                                                                          |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **2026-07-08 12:12:33**            | `remote_transceiver` immediately crashed with exit code 127 because `libmongocxx.so._noabi` was missing. It never restarted.                                                                     |
| **2026-07-08 12:12:36**            | The persisted global path initially could not load because GPS was unavailable to `navigate`. It reported: `No persisted global path could be loaded`. This recovered two seconds later.         |
| **2026-07-08 12:31:00**            | First local-planner failure. Both RRT* attempts rejected the start state as invalid, so navigation published heading `0.0`, set `sail == False`, and the wingsail command dropped to `0.0`.      |
| **2026-07-08 12:41:14**            | The same invalid-start-state failure returned and became permanent.                                                                                                                              |
| **2026-07-08 12:41:14 – 13:02:30** | **639 consecutive failed planning cycles**. Every cycle attempted RRT* twice, failed with `There are no valid initial states!`, and disabled sailing. No recovery occurred before the log ended. |
| **2026-07-08 13:02:30**            | Final recorded planner failure. The log ends two seconds later with navigation still disabled.                                                                                                   |

In total, there were **640 failed planning cycles and 1,280 failed OMPL attempts**. The message saying the path “couldn't be solved within 2” is misleading. OMPL was not merely timing out. It explicitly rejected the start state as invalid.

## Successes

| Timestamp                          | Success                                                                                                                                            |
| ---------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| **2026-07-08 12:12:33**            | CAN transceiver started in production mode and began receiving GPS, wind, temperature and salinity data.                                           |
| **2026-07-08 12:12:33**            | Local transceiver started and created all four services: `send_data`, `debug_send_data`, `check_signal_quality`, and `receive_and_pub`.            |
| **2026-07-08 12:12:35**            | Wingsail controller completed initialization.                                                                                                      |
| **2026-07-08 12:12:36**            | Rosbag successfully opened its database and subscribed to GPS, filtered wind, AIS and local-path topics.                                           |
| **2026-07-08 12:12:38**            | The persisted global path successfully loaded from `main_global_path.csv`.                                                                         |
| **2026-07-08 12:12:38**            | First local path was successfully generated. Navigation began publishing a valid desired heading with `sail == True`.                              |
| **2026-07-08 12:12:38 – 12:30:58** | Navigation operated continuously with sailing enabled.                                                                                             |
| **2026-07-08 12:25:18**            | Existing path intersected an AIS collision zone. The planner detected it, generated a replacement path, and continued sailing.                     |
| **2026-07-08 12:31:02**            | Navigation recovered from the brief 12:31:00 planner failure and resumed sailing.                                                                  |
| **2026-07-08 12:33:18**            | Boat was 0.12 km from its path segment. A new path was successfully generated.                                                                     |
| **2026-07-08 12:39:04**            | Boat was 0.19 km from its path segment. A new path was successfully generated. This was the final successful replan.                               |
| **2026-07-08 12:41:14**            | When navigation failed permanently, the safety response worked: desired heading became `0.0`, `sail == False`, and wingsail commands became `0.0`. |

There were approximately **77 successful local-path generations** before the permanent failure.

Sensor streams remained active almost until the end:

* GPS: **12:12:33 – 13:02:32**
* Wind: **12:12:33 – 13:02:32**
* Temperature and salinity: **12:12:33 – 13:02:31**
* pH: **12:12:34 – 13:02:32**

## Weird or suspicious behaviour

| Timestamp                          | Behaviour                                                                                                                                                                                                                                                                                                            |
| ---------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **2026-07-08 12:12:33**            | Mock AIS was enabled in production, so navigation used simulated AIS instead of real AIS. The visualizer was also disabled in production.                                                                                                                                                                            |
| **2026-07-08 12:12:33 – 13:02:32** | The CAN AIS parser produced **5,772 `ROT is out of bounds` warnings**. ROT was repeatedly `-128`, outside the configured range of `-126` to `126`. Some corresponding AIS packets also contained heading `511`, IDs of `0`, latitude `-90`, longitude `-180` or `100.496`, indicating invalid or unavailable fields. |
| **2026-07-08 12:12:42 – 13:02:16** | AIS assembly reported **314 `Mismatched numShips` resets**, with inconsistent expected and received ship counts.                                                                                                                                                                                                     |
| **2026-07-08 12:12:36 – 13:02:32** | All **14,932 GPS messages received by navigation had heading `0.0`**. This is suspicious unless GPS heading was intentionally unavailable. It could also affect apparent-wind or wingsail calculations.                                                                                                              |
| **2026-07-08 12:12:38**            | The log says `Reached final global waypoint; switching to index 1` immediately after loading the path. The wording is contradictory and may be a misleading log message or waypoint-indexing issue.                                                                                                                  |
| **2026-07-08 12:13:09**            | The final `[MAIN TRIM TAB]` CAN message was received. Wingsail continued publishing commands until **13:02:31**, but no further trim-tab CAN reports appeared. This may indicate loss of actuator feedback or telemetry.                                                                                             |
| **2026-07-08 12:13:36 – 12:39:04** | Paths were replaced frequently. Median active-path lifetime was about **10 seconds**, with **19 paths lasting five seconds or less**. Most changes were triggered by wind-direction changes barely exceeding the 10-degree threshold.                                                                                |
| **2026-07-08 12:31:02**            | After the transient planner failure, it logged `Target waypoint index out of bounds: 1 >= 0`, but then immediately selected a waypoint and resumed sailing. That state transition deserves investigation.                                                                                                            |
| **2026-07-08 13:02:32**            | The log ends while nodes are still actively publishing. There are no shutdown, SIGINT or clean-exit messages, so the recording appears to stop abruptly.                                                                                                                                                             |

## Main conclusion

The run had three major problems:

1. **Remote communications were unavailable for the entire run** because `remote_transceiver` crashed at startup.
2. **Real AIS decoding was extremely noisy or malformed**, although mock AIS prevented it from directly breaking pathfinding.
3. **The local planner became unusable at 12:41:14** because its current start state was considered invalid. Navigation remained safely disabled for the final **21 minutes 16 seconds** of the run.

The highest-priority investigation is why the validity checker rejected the boat’s current state beginning at **2026-07-08 12:41:14**.
