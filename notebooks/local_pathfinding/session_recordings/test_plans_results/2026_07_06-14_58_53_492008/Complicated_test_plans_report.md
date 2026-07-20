# Test-Plan Result and Analysis

## Overall summary

This batch of tests is `2026_07_06-14_58_53_492008` containing 12 sequential runs recorded from 2026-07-06 14:58:53 through 2026-07-07 13:25:29, totalling 80,795.557 seconds (22:26:35.557). All 12 launch processes ran: 1 completed and 11 reached the 2-hour timeout.

Only 5 recordings are valid tests of their named YAML scenarios: 1 passed and 4 were partial/inconclusive. The other 7 used a persisted route or the wrong active waypoint index and therefore do not assess their named plans.

System health was consistent across the batch: every `result.json` reports all expected nodes present, `return_code: 0`, and a stored launch log. Errors after timeout or completion were controlled-shutdown artifacts. No route-valid run logged an OMPL no-solution failure or unexpected node crash.

> **Note — global_path-adoption bug:** 7 runs used incorrect `global_path`. There is a non-deterministic behavior detected from a second run of these same 12 tests, indicating a probable ROS startup/discovery race condition between `mock_global_path` and `navigate_main`. During normal startup, `mock_global_path` publishes the path once, while `navigate_main` fails to recevice the new path because of publisher–subscriber discovery is incomplete. The message may not be delivered and the navigation may continue using the persisted global_path from the respected csv file.

## Test plan run outcomes

A run is route-valid only when navigator and monitor clearly used the same waypoint list and starting leg. For invalid runs, timeout remains an operational fact, but progress, completion distance, path efficiency, maneuvers, and avoidance are not attributable to the named YAML.

Eleven runs ended because `Test plan exceeded timeout of 2 hours.` Jericho ended because `Reached final Global waypoint before timeout.`

### Route-valid runs

| Test plan | Scenario configuration | Route alignment | Outcome and assessment |
|---|---|---|---|
| `jericho_on_water_test` | <ul><li>**Wind:** `270°` at `14 km/h`</li><li>**Route:** 5-waypoint Jericho/English Bay loop</li><li>**AIS:** None configured</li></ul> | New 5-waypoint route started at index 4. | **COMPLETED — valid pass.**<br>Duration: 00:25:31.889.<br>Final index 0 reached within 299.929 m; 0 km remained. |
| `launch` | <ul><li>**Wind:** `270°` at `14 km/h`</li><li>**Route:** 7-waypoint offshore return loop</li><li>**AIS:** None configured</li></ul> | New 7-waypoint route started at index 6. | **TIMEOUT — valid partial/inconclusive.**<br>Duration: 02:00:05.868.<br>Advanced from index 6 to index 4; finished 2.655 km from the active waypoint with 14.016 km remaining. Normal progress, but the route exceeded the 2-hour window. |
| `multi_ship_convergence` | <ul><li>**Wind:** `180°` at `12 km/h`</li><li>**Route:** 2 waypoints</li><li>**AIS:** 3 converging ships: nominal head-on, starboard-bow give-way, and port-bow stand-on encounters</li></ul> | New 2-waypoint route started at index 1. | **TIMEOUT — valid partial/inconclusive.**<br>Duration: 02:00:05.858.<br>Remained at index 1, 22.339 km away, **with** 33.254 km remaining. Progress was smooth, but the artifacts do not certify per-ship COLREG compliance. |
| `near_land` | <ul><li>**Wind:** `180°` at `10 km/h`</li><li>**Route:** 2-waypoint westbound route</li><li>**AIS:** 1 oncoming ship</li><li>**Land:** 2 polygons around the route</li></ul> | New 2-waypoint route started at index 1. | **TIMEOUT — valid partial/inconclusive.**<br>Duration: 02:00:05.785.<br>Finished 2.025 km from index 1 with 12.938 km remaining. Land-constrained progress was smooth, but the first target was not reached. |
| `vancouver_to_gulf_of_alaska_complex` | <ul><li>**Wind:** `145°` at `5 km/h`</li><li>**Route:** 10-waypoint Vancouver departure route</li><li>**AIS:** 14 contacts</li><li>**Land:** 7 land/shoal polygons</li></ul> | New 10-waypoint route adopted; start-coincident index 9 cleared and index 8 became active. | **TIMEOUT — valid partial/inconclusive.**<br>Duration: 02:00:05.725.<br>Index 8 remained 549.788 m away, with 24.596 km remaining. The approximately 27 km route could not finish within 2 hours at the observed speed, and a first-leg detour reduced progress. |

Scenario-definition warnings: Jericho and `launch` describe shoreline land in comments, but their stored YAMLs have no explicit `land` key/list. Vancouver's wind comment conflicts with its configured `145°` at `5 km/h` value.

### Route-invalid runs

These runs confirm that the launch process operated until timeout, but they do not evaluate the behavior or performance of the named scenarios. The route-mismatch evidence below explains why; a separate "Named behavior not assessed" column would repeat this same conclusion.

| Test plan | Scenario configuration<br>**(YAML)** | Route-mismatch evidence | Operational outcome |
|---|---|---|---|
| `basic_more_obstacles` | <ul><li>**Wind:** `0°` at `10 km/h`</li><li>**Route:** 2-point long-range offshore route</li><li>**AIS:** 7 ships</li><li>**Land:** 4 polygons</li></ul> | The persisted and intended routes contained the same 2 waypoints, but the navigator resumed at index 0 while the GoalMonitor evaluated index 1. | **TIMEOUT**<br>Duration: 02:00:05.711 |
| `gauntlet` | <ul><li>**Wind:** `180°` at `10 km/h`</li><li>**Route:** Westbound corridor</li><li>**AIS:** 7 ships on mixed courses</li></ul> | Navigator followed the persisted `basic_more_obstacles` route and targeted `(48.158390, -130.253906)`. | **TIMEOUT**<br>Duration: 02:00:05.814 |
| `low_wind_tack` | <ul><li>**Wind:** `270°` at `1.5 km/h`</li><li>**Route:** Westbound upwind route</li><li>**Objective:** Recover from irons, then make upwind progress</li></ul> | Navigator followed the persisted 7-waypoint `launch` route and targeted `(48.798546, -125.235504)`. | **TIMEOUT**<br>Duration: 02:00:05.651 |
| `narrow_channel` | <ul><li>**Wind:** `180°` at `12 km/h`</li><li>**Route:** Narrow east-west channel</li><li>**Land:** 2 rectangles forming the channel</li></ul> | The persisted and intended waypoint coordinates matched, but the navigator resumed at index 0 while the monitor evaluated index 1. | **TIMEOUT**<br>Duration: 02:00:05.893 |
| `pinched` | <ul><li>**Wind:** `180°` at `12 km/h`</li><li>**Route:** Corridor between a northern coastline and a parallel vessel</li><li>**AIS:** 1 slower parallel ship</li><li>**Land:** Northern coastline</li></ul> | Navigator followed the persisted `near_land` route and targeted `(49.285000, -123.449997)`. | **TIMEOUT**<br>Duration: 02:00:05.774 |
| `upwind_narrow_channel` | <ul><li>**Wind:** `270°` at `15 km/h`</li><li>**Route:** Short upwind route through a narrow channel</li><li>**Objective:** Tack through the channel</li></ul> | Navigator followed the persisted `near_land` route and targeted `(49.285000, -123.449997)`. | **TIMEOUT**<br>Duration: 02:00:05.740 |
| `upwind_tack` | <ul><li>**Wind:** `270°` at `15 km/h`</li><li>**Route:** Westbound upwind route</li><li>**AIS:** 1 crossing ship</li><li>**Objective:** Upwind tacking and crossing avoidance</li></ul> | Navigator followed the persisted `near_land` route and targeted `(49.285000, -123.449997)`. | **TIMEOUT**<br>Duration: 02:00:05.849 |

The invalid `gauntlet` hybrid recording is still diagnostically useful: it contains 2,699 replan rows and 111 complete two-attempt OMPL failures followed by sail disablement. These observations are not attributable to the named gauntlet scenario.

## Quantitative evidence for route-valid runs

The rosbag recordings were analysed through the executed `explore_bag_data` notebook exports.

### Performance

| Test plan | Cycles | Duration (h) | Travelled (km) | Straight (km) | Distance ratio | Avg track speed (km/h) | Mean filtered wind (km/h) | Max boat speed (km/h) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `jericho_on_water_test` | 762 | 0.423 | 2.770 | 0.444 | 6.231 | 6.551 | 14.922 | 6.962 |
| `launch` | 3,599 | 1.999 | 12.973 | 5.806 | 2.235 | 6.490 | 12.124 | 6.962 |
| `multi_ship_convergence` | 3,600 | 1.999 | 12.146 | 11.842 | 1.026 | 6.076 | 13.495 | 6.284 |
| `near_land` | 3,600 | 1.999 | 10.356 | 9.679 | 1.070 | 5.179 | 11.225 | 5.495 |
| `vancouver_to_gulf_of_alaska_complex` | 3,599 | 1.999 | 5.219 | 2.820 | 1.851 | 2.611 | 6.490 | 2.575 |

`Cycles` counts `/local_path` records. `Travelled` is integrated GPS-track distance; `Straight` is start-to-end displacement; their quotient is `Distance ratio`. `Avg track speed` is track distance divided by notebook duration, not the GPS speed-field mean. `Mean filtered wind` is filtered apparent wind, not the YAML's injected true wind; the notebook combines it with GPS heading and speed to compute true wind.

Jericho and `launch` follow loop-like routes, so their straight-line ratios are not simple inefficiency scores. Vancouver's integrated average slightly exceeds its sampled maximum because summed GPS drift/noise inflates track length; mock-GPS positions in its launch log give a better approximate average of 2.31 km/h.

### Time normalization

Actual durations are listed in the performance table.

| Test plan | Ideal using max boat speed (h) | Actual / ideal | Ideal using mean filtered wind (h) | Actual / ideal |
|---|---:|---:|---:|---:|
| `jericho_on_water_test` | 0.064 | 6.622 | 0.030 | 14.193 |
| `launch` | 0.834 | 2.397 | 0.479 | 4.174 |
| `multi_ship_convergence` | 1.885 | 1.061 | 0.878 | 2.278 |
| `near_land` | 1.761 | 1.135 | 0.862 | 2.319 |
| `vancouver_to_gulf_of_alaska_complex` | 1.095 | 1.825 | 0.434 | 4.600 |

Ideal times use achieved start-to-end displacement, not complete route length. They are normalizations rather than complete sailing models.

### Planner cycles and maneuvers

| Test plan | Path switches | Heading changes | Changes/cycle | Tacks | Gybes | Ambiguous | Replans and reasons |
|---|---:|---:|---:|---:|---:|---:|---|
| `jericho_on_water_test` | 4 | 11 | 0.014436 | 2 | 4 | 3 | 5 new-global-waypoint |
| `launch` | 12 | 14 | 0.003890 | 7 | 0 | 2 | 10 TTL + 3 new-global-waypoint |
| `multi_ship_convergence` | 12 | 2 | 0.000556 | 0 | 0 | 0 | 11 TTL + 1 collision-zone + 1 new-global-waypoint |
| `near_land` | 11 | 7 | 0.001944 | 0 | 0 | 0 | 11 TTL + 1 new-global-waypoint |
| `vancouver_to_gulf_of_alaska_complex` | 11 | 11 | 0.003056 | 0 | 4 | 0 | 11 TTL + 1 new-global-waypoint |

A `Path switch` is a change in the rounded local-path signature, not every replan request. A `Heading change` is a desired-heading change greater than 1 degree. `Tack`, `gybe`, and `ambiguous` are debounced desired-heading side changes relative to notebook-computed true wind; they describe commanded maneuvers and do not independently prove physical completion. A `Replan` is a non-empty reason reported in `LPathData`.

## Qualitative evidence for route-valid runs

| Test plan | Path, heading, and wind behavior | AIS, land, replanning, and interpretation evidence |
|---|---|---|
| `jericho_on_water_test` | <ul><li>**Path:** Continuous loop without sustained spatial trapping or rapid oscillation.</li><li>**Heading:** Desired-heading changes mainly coincided with waypoint progression.</li><li>**Wind:** Sailing- and true-wind-angle plots show mixed points of sail and the reported maneuver types.</li></ul> | <ul><li>**Replanning:** All replans were waypoint progression; no collision or TTL replan occurred.</li></ul> |
| `launch` | <ul><li>**Path:** Regular alternating legs without sustained spatial trapping.</li><li>**Heading:** Changes aligned with waypoint and TTL replans rather than continuous high-frequency switching.</li><li>**Wind:** Commanded tacks followed the alternating legs.</li></ul> | <ul><li>**Replanning:** No collision-triggered replan occurred.</li></ul> |
| `multi_ship_convergence` | <ul><li>**Path:** Nearly straight, without oscillation or trapped behavior after TTL replans.</li><li>**Heading:** Desired heading stayed close to west.</li><li>**Wind:** The track remained on one side of computed true wind.</li></ul> | <ul><li>**AIS:** An initial collision-zone correction was followed by no later collision invalidation.</li><li>**Replanning:** Subsequent replans were TTL-triggered.</li><li>**Interpretation:** Exports provide no per-ship CPA or COLREG decisions.</li></ul> |
| `near_land` | <ul><li>**Path:** Smooth westbound curve without sustained oscillation or trapped behavior.</li><li>**Heading:** Stepwise desired-heading adjustments followed replans.</li><li>**Wind:** Sailing angle remained on one side of computed true wind, with no detected maneuver.</li></ul> | <ul><li>**Land:** Visual inspection shows the track outside the configured polygons.</li><li>**Replanning:** No collision-triggered invalidation occurred.</li><li>**Interpretation:** Clearance was not tested with a numerical point-in-polygon check.</li></ul> |
| `vancouver_to_gulf_of_alaska_complex` | <ul><li>**Path:** Short northerly detours alternated with westward legs; the first-leg detour slowed progress without repeated spatial oscillation or a trapped planner.</li><li>**Heading:** Desired headings alternated with the detour and westward legs.</li><li>**Wind:** Commanded gybes aligned with TTL replans while sailing mostly at broad-reach/downwind angles.</li></ul> | <ul><li>**AIS:** The notebook recorded 14 contacts; configured head-on, crossing, and overtaking encounters lack exported per-contact CPA/COLREG decisions.</li><li>**Land:** The notebook recorded 21 obstacles in total (14 AIS plus 7 land/shoal polygons). Visual inspection shows clearance from nearby departure polygons, but farther corridor obstacles were not reached and clearance was not tested numerically.</li><li>**Replanning:** No collision- or wind-triggered regeneration occurred.</li></ul> |

Boat-heading, true-wind, and sailing-angle plots were reviewed qualitatively; aggregate numeric distributions were not exported.

## Final Notes

Evidence reviewed comprised batch `summary.json`, each run's `result.json`, rendered `explore_bag_data.html`, and `launch.log`, plus the corresponding YAML under `src/local_pathfinding/test_plans/`. Each outcome-table row links its YAML, result, log, notebook, and rosbag.

All runs used a 2-hour timeout, `development` mode, `globals.yaml`, `info` logging, rosbag recording, and output root `notebooks/local_pathfinding/session_recordings/test_plans_results/2026_07_06-14_58_53_492008/<test-name>`.

Recorded/Used command:

```text
ros2 run local_pathfinding run_test_plans -t 2 4 6 8 9 10 11 12 16 17 18 19 -n 12 --timeout_hours 2
```

Across the valid 2-hour runs, regeneration generally followed the roughly 10-minute TTL. No significant-wind replan occurred. The available evidence supports smooth progress and nearby obstacle clearance where stated, but does not extend to unexported per-contact COLREG decisions, numerical polygon-clearance tests, or route segments not reached.

**Action:** fix the global-path adoption race, make `run_test_plans` verify that `node_navigate` adopted the selected YAML route before timing begins, and then rerun `basic_more_obstacles`, `gauntlet`, `low_wind_tack`, `narrow_channel`, `pinched`, `upwind_narrow_channel`, and `upwind_tack`.
