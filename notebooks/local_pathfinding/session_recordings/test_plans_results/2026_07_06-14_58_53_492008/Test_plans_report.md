<style>
.wide-table {
  width: 100%;
  overflow-x: auto;
}

.wide-table table {
  min-width: 1800px;
}

.wide-table th,
.wide-table td {
  min-width: 180px;
}
</style>

# Test Plan Results and Analysis

## Overall Summary
<!-- markdownlint-disable MD013 -->
**Batch:** `2026_07_06-14_58_53_492008` <br>
**Recorded:** `2026-07-06 14:58:53` to `2026-07-07 13:25:29`

- **Test Runs:**<br>
  12 tests ran for approximately 22.5 hours. 1 passed and 4 were partial/inconclusive, the remaining 7 used a persisted[^"persisted-route"] rotue or the wrong active waypoint index and therefore they do not assess their named test plans.

- **System health:**<br>
  All expected nodes ran successfully. No route-valid run had an unexpected crash or OMPL no-solution failure.

<br>

> **Note — global_path-adoption bug:** <br>
> 7 runs used incorrect `global_path`. There is a non-deterministic behavior detected indicating a probable ROS startup/discovery race condition between `mock_global_path` and `navigate_main`.<br>
> During normal startup, `mock_global_path` publishes the path once, while `navigate_main` fails to receive the new path probably because publisher–subscriber discovery is incomplete. Resulting in `node_navigate.py` using persisted[^"persisted-route"] global_path.

<br>

## Test Plans Outcomes

- **Route-Valid:** A run is counted route-valid only when both the `GoalMonitor`[^"GoalMonitor"] and `node_navigate.py` used the same configuration mentioned in the respective test plan's YAML file.

- **Route-Invalid:** A run is counted route-invalid when it doesn't follow Route-Valid's defintion.
    - In this case, status[^"status"] remains an operational fact, but progress, completion distance, path efficiency, maneuvers, and avoidance can not be attributed to the respective test plan.

<br>

### Route-Valid Runs

<div class="wide-table">

| Test plan | Configuration | Route description <br> (`global_path` setting) | Outcome | Assessment |
| --------- | ------------- | -------------------------------------------- | ------- | ---------- |
| `jericho_on_water_test` | <ul><li>**True Wind:** `270°` at `14 km/h`</li><li>**`global_path`:** 5-waypoint Jericho/English Bay loop</li><li>**AIS:** N/A</li></ul> | 5-waypoint route started at index 4. | **Status: COMPLETED**<ul><li>Duration: 00:25:31.889</li><li>Reached within 299.929 m of final waypoint</li><li>Total remaining distance: 0 km</li></ul> | Test completed successfully |
| `launch` | <ul><li>**True Wind:** `270°` at `14 km/h`</li><li>**`global_path`:** 7-waypoint offshore return loop</li><li>**AIS:** N/A</li></ul> | 7-waypoint route started at index 6. | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.868</li><li>Advanced from index 6 to index 4</li><li>Stopped at 2.655 km from the active waypoint</li><li>Total remaining distance: 14.016 km</li></ul> | Normal progress. |
| `multi_ship_convergence` | <ul><li>**True Wind:** `180°` at `12 km/h`</li><li>**`global_path`:** 2 waypoints</li><li>**AIS:** 3 converging ships: <ul><li>nominal head-on</li><li> starboard-bow give-way</li><li>port-bow stand-on encounters</li></ul></li></ul> | 2-waypoint route started at index 1. | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.858</li><li>Remained at index 1</li><li>Stopped at 22.339 km from waypoint at index 1</li><li>Total remaining distance: 33.254 km</li></ul> | Progress was smooth, but the artifacts do not certify per-ship COLREG compliance. |
| `near_land` | <ul><li>**True Wind:** `180°` at `10 km/h`</li><li>**`global_path`:** 2-waypoint westbound route</li><li>**AIS:** 1 oncoming ship</li><li>**Land:** 2 polygons around the route</li></ul> | 2-waypoint route started at index 1. | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.785</li><li>Stopped at 2.025 km from waypoint at index 1</li><li>Total remaining distance: 12.938 km</li></ul> | Land-constrained progress was smooth. |
| `vancouver_to_gulf_of_alaska_complex` | <ul><li>**True Wind:** `145°` at `5 km/h`</li><li>**`global_path`:** 10-waypoint Vancouver departure route</li><li>**AIS:** 14 contacts</li><li>**Land:** 7 land/shoal polygons</li></ul> | <ul><li>10-waypoint route started at index 9</li><li>index 9 cleared and index 8 became active.</li></ul> | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.725</li><li>Stopped at 5.49788 km from waypoint at index 8</li><li>Total remaining distance: 24.596 km</li></ul> | The approximately 27 km route could not finish within 2 hours at the observed speed, and a first-leg detour reduced progress. |

</div>

>Note: `vancouver_to_gulf_of_alaska_complex`'s wind comment conflicts with its configured `145°` at `5 km/h` value.

<br>

### Route-Invalid Runs

<div class="wide-table">

| Test plan | Configuration | Route description <br> (`global_path` setting) | Outcome <br> (data captured by `GoalMonitor`) | Assessment |
| --------- | ------------- | -------------------------------------------- | --------------------------------------------- | ---------- |
| `basic_more_obstacles` | <ul><li>**True Wind:** `0°` at `10 km/h`</li><li>**`global_path`:** 2-waypoint, Long-range offshore route</li><li>**AIS:** 7 ships</li><li>**Land:** 4 polygons</li></ul> | the navigation started at index 0 (which is the final global waypoint) while the `GoalMonitor`[^"GoalMonitor"] evaluated index 1 of the correct `global_path` | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.711</li><li>Stopped at 1016.0191260300227 km from waypoint at index 1</li><li>Total remaining distance: 1519.648945435294 km</li></ul> | `node_navigate.py` used a persisted[^"persisted-route"] path, causing it to deliberately choose waypoint index 0 as its starting target. |
| `gauntlet` | <ul><li>**True Wind:** `180°` at `10 km/h`</li><li>**`global_path`:** 2-waypoint, Westbound corridor</li><li>**AIS:** 7 ships on mixed courses</li></ul> | `node_navigate.py` used the persisted[^"persisted-route"] route and targeted `(48.158390, -130.253906)` at index 0. While `GoalMonitor`[^"GoalMonitor"] evaluated index 1 of the correct `global_path` | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.814</li><li>Stopped at 21.155446045802524 km from waypoint at index 1</li><li>Total remaining distance: 28.431520438441054 km</li></ul> | `node_navigate.py` used the same persisted[^"persisted-route"] path as `basic_more_obstacles`, causing it to deliberately choose waypoint index 0 as its starting target. |
| `low_wind_tack` | <ul><li>**True Wind:** `270°` at `1.5 km/h`</li><li>**`global_path`:** 2-waypoint, Westbound upwind route</li><li>**AIS:** N/A</li></ul> | <ul><li>Recover from irons, then make upwind progress</li><li>`node_navigate.py` used the persisted[^"persisted-route"] route and targeted `(48.798546, -125.235504)` at index 0. While `GoalMonitor`[^"GoalMonitor"] evaluated index 1 of the correct `global_path`</li></ul> | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.651</li><li>Stopped at 30.454918420516315 km from waypoint at index 1</li><li>Total remaining distance: 41.369029014816675 km</li></ul> | `node_navigate.py` used the same persisted[^"persisted-route"] path as `launch` and targeted `(48.798546, -125.235504)`, causing it to deliberately choose waypoint index 0 as its starting target. |
| `narrow_channel` | <ul><li>**True Wind:** `180°` at `12 km/h`</li><li>**`global_path`:** 2-waypoint, Narrow east-west channel</li><li>**Land:** 2 polygons forming the channel</li><li>**AIS:** N/A</li></ul> | `node_navigate.py` used the persisted[^"persisted-route"] route and targeted `(49.279998779296875, -123.5)` at index 0. While `GoalMonitor`[^"GoalMonitor"] evaluated index 1 of the correct `global_path` | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.893</li><li>Stopped at 24.50935277406248 km from waypoint at index 1</li><li>Total remaining distance: 35.423463368363875 km</li></ul> | `node_navigate.py` used the same persisted[^"persisted-route"] path as `multi_ship_convergence` and targeted `(49.279998779296875, -123.5)`, causing it to deliberately choose waypoint index 0 as its starting target. |
| `pinched` | <ul><li>**True Wind:** `180°` at `12 km/h`</li><li>**`global_path`:** 2-waypoint Corridor between a northern coastline and a parallel vessel</li><li>**AIS:** 1 slower, parallel ship</li><li>**Land:** 1 polygon, Northern coastline</li></ul> | `node_navigate.py` used the persisted[^"persisted-route"] route and targeted `(49.285000, -123.449997)` at index 0. While `GoalMonitor`[^"GoalMonitor"] evaluated index 1 of the correct `global_path` | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.774</li><li>Stopped at 22.47499773727005 km from waypoint at index 1</li><li>Total remaining distance: 33.39131488970042 km</li></ul> | `node_navigate.py` used the same persisted[^"persisted-route"] path as `near_land` and targeted `(49.285000, -123.449997)`, causing it to deliberately choose waypoint index 0 as its starting target. |
| `upwind_narrow_channel` | <ul><li>**True Wind:** `270°` at `15 km/h`</li><li>**`global_path`:** 2-waypoint, Short upwind route through a narrow channel</li><li>**AIS:** N/A</li></ul> | <ul><li>Tack through the channel</li><li>`node_navigate.py` used the persisted[^"persisted-route"] route and targeted `(49.285000, -123.449997)` at index 0. While `GoalMonitor`[^"GoalMonitor"] evaluated index 1 of the correct `global_path`</li></ul> | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.740</li><li>Stopped at 29.8210872179707 km from waypoint at index 1</li><li>Total remaining distance: 40.73519781227209 km</li></ul> | `node_navigate.py` used the same persisted[^"persisted-route"] path as `near_land` and targeted `(49.285000, -123.449997)`, causing it to deliberately choose waypoint index 0 as its starting target. |
| `upwind_tack` | <ul><li>**True Wind:** `270°` at `15 km/h`</li><li>**`global_path`:** 2-waypoint, Westbound upwind route</li><li>**AIS:** 1 crossing ship</li></ul> | <ul><li>Upwind tacking and crossing avoidance</li><li>`node_navigate.py` used the persisted[^"persisted-route"] route and targeted `(49.285000, -123.449997)` at index 0. While `GoalMonitor`[^"GoalMonitor"] evaluated index 1 of the correct `global_path`</li></ul> | **Status: TIMEOUT**<ul><li>Duration: 02:00:05.849</li><li>Stopped at 15.411499234114459 km from waypoint at index 1</li><li>Total remaining distance: 26.387266451563974 km</li></ul> | `node_navigate.py` used the same persisted[^"persisted-route"] path as `near_land` and targeted `(49.285000, -123.449997)`, causing it to deliberately choose waypoint index 0 as its starting target. |

</div>

> Note: <br>
>These observations can not be attributed to their respective test plan<br>
>`gauntlet` was the only test among the 12 ran tests that published `sail == False`, doing so 111 times.

<br>
<br>

## Rosbag Recordings Analysis

These rosbag recording were analyzed using `explore_bag_data.ipynb` notebook. <br>
We are gonna do this analysis for only the meaningful test runs i.e., Route-Valid test plans.

### Performance

<div class="wide-table">

| Test plan | Cycles | Duration (h) | Travelled (km) | Straight (km) | Distance ratio | Avg track speed (km/h) | Mean filtered wind (km/h) | Max boat speed (km/h) |
| --------- | -----: | -----------: | -------------: | ------------: | -------------: | ---------------------: | ------------------------: | ---------------------: |
| `jericho_on_water_test` | 762 | 0.423 | 2.770 | 0.444 | 6.231 | 6.551 | 14.922 | 6.962 |
| `launch` | 3,599 | 1.999 | 12.973 | 5.806 | 2.235 | 6.490 | 12.124 | 6.962 |
| `multi_ship_convergence` | 3,600 | 1.999 | 12.146 | 11.842 | 1.026 | 6.076 | 13.495 | 6.284 |
| `near_land` | 3,600 | 1.999 | 10.356 | 9.679 | 1.070 | 5.179 | 11.225 | 5.495 |
| `vancouver_to_gulf_of_alaska_complex` | 3,599 | 1.999 | 5.219 | 2.820 | 1.851 | 2.611 | 6.490 | 2.575 |

</div>

- **Cycles:** Number of `/local_path` messages recorded during the test
- **Duration:** Elapsed time between the first and last `/local_path` timestamps.
- **Travelled:** Estimated total distance travelled by the boat.
- **Straight:** Direct distance from the first recorded GPS position to the final recorded position.
- **Distance ratio:** Compares the `travelled` path with the `straight`
- **Avg track speed:** Average speed estimated from the recorded GPS track
- **Mean filtered wind:** Average magnitude reported by `/filtered_wind_sensor`.

>Note:<br>
>`jericho_on_water_test` and `launch` test plans follow loop-like routes, so their `distance ratio` values should not be interpreted as simple inefficiency scores. <br>
>`vancouver_to_gulf_of_alaska_complex` test plan's `Avg track speed` of `2.611 km/h` slightly exceeds its maximum sampled speed of `2.575 km/h` because of the accumulated GPS drift and noise inflating the integrated track distance.

<br>

### Planner cycles and maneuvers

<div class="wide-table">

| Test plan | Path switches | Heading changes | Heading Changes / cycle | Tacks | Gybes | Ambiguous | Replans and reasons |
| --------- | ------------: | --------------: | ----------------------: | ----: | ----: | --------: | ------------------- |
| `jericho_on_water_test` | 4 | 11 | 0.014436 | 2 | 4 | 3 | 5 new-global-waypoint |
| `launch` | 12 | 14 | 0.003890 | 7 | 0 | 2 | 10 TTL + 3 new-global-waypoint |
| `multi_ship_convergence` | 12 | 2 | 0.000556 | 0 | 0 | 0 | 11 TTL + 1 collision-zone + 1 new-global-waypoint |
| `near_land` | 11 | 7 | 0.001944 | 0 | 0 | 0 | 11 TTL + 1 new-global-waypoint |
| `vancouver_to_gulf_of_alaska_complex` | 11 | 11 | 0.003056 | 0 | 4 | 0 | 11 TTL + 1 new-global-waypoint |

</div>

- **Path Switch:** Shows how often the generated local path changed.
- **Heading change:** Shows how often the desired-heading changed by more than `1°`.
- **Heading change / Cycle:** Number of heading changes divided by the number of `/local_path` records.
- **Tack, gybe, and ambiguous:** Debounced desired-heading side changes relative to notebook-estimated true wind. They are inferred commanded maneuvers and do not prove physical completion.
- **Replan and reasons:** A `/local_path` record containing the `replan_reason`.

<br>

## Qualitative evidence for route-valid runs

Boat-heading, true-wind, and sailing-angle plots were reviewed qualitatively; aggregate numeric distributions were not exported.

<div class="wide-table">

| Test plan | Path behavior | Heading behavior | Wind behavior | AIS evidence | Land evidence | Replanning evidence | Interpretation |
| --------- | ------------- | ---------------- | ------------- | ------------ | ------------- | ------------------- | -------------- |
| `jericho_on_water_test` | The boat followed a loop and kept moving without getting stuck or repeatedly moving back and forth. | Most major direction changes happened as the boat moved to the next global waypoint. | The boat was commanded to sail at several angles to the wind; the notebook identified 2 tacks, 4 gybes, and 3 unclear transitions. | — | — | All 5 replans happened when the boat moved to a new global waypoint. None were caused by a collision risk or an expired local path. | — |
| `launch` | The boat followed repeated zigzag legs and did not get stuck. | Direction changes generally happened when the route advanced to a new waypoint or the local path expired and was replanned. | The notebook identified 7 commanded tacks as the boat changed between alternating legs. | — | — | The run had 10 replans because the local path expired and 3 because the boat moved to a new global waypoint. None were caused by a collision risk. | — |
| `multi_ship_convergence` | The boat travelled almost straight west without getting stuck or repeatedly turning after replans. | The commanded direction stayed close to west and changed by more than `1°` only twice. | The commanded heading stayed on the same side of the estimated true wind, so no tack or gybe was detected. | The planner requested one replan near the start because the path entered a collision zone. No later replan was caused by a collision zone. | — | After the initial events, the remaining 11 replans happened because the local path expired. | `explore_bag_data` did not show the closest approach or avoidance decision for each ship, so they cannot confirm compliance with collision regulations. |
| `near_land` | The boat followed a smooth westbound curve without getting stuck or repeatedly moving back and forth. | The commanded direction changed in clear steps after some replans. | The commanded heading stayed on the same side of the estimated true wind, so no tack or gybe was detected. | — | Land was configured, but the position plot does not show the land polygons. Therefore we cannot confirm the boat's clearance from land. | The run had 11 replans because the local path expired and 1 because a new global waypoint was received. None were caused by a collision zone. | The available analysis did not calculate the boat's distance from land. |
| `vancouver_to_gulf_of_alaska_complex` | The boat alternated between short northward detours and westward legs. The first detour reduced westward progress, but the boat did not become stuck or repeat the same movement. | The commanded direction alternated between the northward detours and westward legs. | The notebook identified 4 commanded gybes during TTL replans, while the boat was mostly commanded to sail with the wind coming from behind or from behind one side. | The notebook recorded 14 AIS contacts. | The notebook recorded 7 land or shallow-water polygons among 21 total obstacles. The position plot does not show these polygons, so it cannot confirm land clearance. | The run had 11 replans because the local path expired and 1 because a new global waypoint was received. None were caused by a collision zone or a wind change. | The `explore_bag_data` did not show the closest approach or avoidance decision for each ship, so the configured encounters cannot be assessed individually. |

</div>

<br>

## Final Notes

- **Resources used:** Batch summary, test-plan YAML, and each run's `result.json`, `launch.log`, notebook analysis on the rosbag recording.

- **Settings for test plan:** 2-hour timeout, `development` mode, `globals.yaml`, `info` logging, and rosbag recording.

### Command Used

```text
ros2 run local_pathfinding run_test_plans -t 2 4 6 8 9 10 11 12 16 17 18 19 -n 12 --timeout_hours 2
```

Across the valid 2-hour runs, re-plans generally followed the roughly 10-minute TTL; none were caused by significant wind changes. The evidence supports smooth progress and reported obstacle clearance, but cannot verify individual COLREG decisions, numerical polygon clearance, or unreached route segments.

[^"persisted-route"]: A persisted route is a previously saved route in the `main_global_path.csv`.
[^"GoalMonitor"]: The class for monitoring test plan progress.
[^"status"]: Status defined the final outcome of the test plan run. It can be:

    - `COMPLETED`: The test plan run ran successfully and the boat reached the final waypoint in the `global_path`.
    - `TIMEOUT`: The test plan could not finish running within the alloted time.
    - `INTERRUPTED`: The test plan was interrupted by the user.
    - `FAILED`: The test plan did not ran successfully. It failed either due to a node crash or some other reason.
