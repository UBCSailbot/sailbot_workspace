# Respawn Verification Voyage Campaign — Summary (#1065)

**Run:** 2026-08-17 09:47Z → 14:23Z · **Budget:** 5 h total (voyages ended when the budget was
consumed) · **Mode:** development · **ROS_DOMAIN_ID:** 55 · **All local — no commits.**
**Under test:** the `navigate_main` auto-respawn fix (`respawn=True`, `respawn_delay=2.0` in
`launch/main_launch.py`) + the startup log marker in `node_navigate.py`.

## TL;DR — the respawn fix works ✅

The exact OMPL segfault that **permanently killed pathfinding before** now triggers an automatic
respawn and the voyage **keeps sailing**. Voyage A hit the segfault at **t+3h49m**, recovered in
seconds, and **sailed on to the full ~4.4 h budget** (it *died permanently at ~4h11m before the
fix*). Across both voyages, **every** navigate segfault (3 total) was caught and recovered — **zero
uncaught / fatal crashes**. The only thing that stopped a voyage early was the *test harness's* own
watchdog, not the respawn (see Finding 2).

## Voyages run (order by replan rate: B → A; C not reached)

| Voyage | Conditions | Ran for | Outcome | Segfaults → respawns | Pathfinding |
|---|---|---|---|---|---|
| **B** — Haida Gwaii gale | gale wind + reroutes + drift, high replan rate | **21 min** | stack torn down by the runner's node-missing watchdog | **2 → 2** (both recovered) | slow upwind, ~1.2 km toward dest before crash cluster |
| **A** — JdF → Pacific | dense AIS + wind + reroutes, moderate replan rate | **~4.4 h (TIMEOUT)** | **survived the full budget** | **1 → 1** (recovered, kept sailing) | steady progress, ~17 km toward dest, 0 PathNotFound |
| **C** — realistic reach | — | not run | A survived (not a permanent crash) → per the rule, C not reached | — | — |

## Respawn behavior (the headline)

- **Voyage A — the clean demonstration.** One OMPL segfault (`exit -11`) at **t+3h49m12s**. Launch
  restarted `navigate_main` within seconds (`respawn_delay=2.0` + ~3–6 s startup, well under the
  runner's 10 s window). By the next 2-min health snapshot navigate was back, publishing
  `desired_heading` again, and **distance-to-destination kept decreasing** (697.2 → 695.0 km after
  the respawn). It then sailed to the budget TIMEOUT. The new startup marker logged **twice**
  (1 initial + 1 respawn), exactly as designed.
- **Voyage B — respawn also worked, twice.** Two segfaults in ~2 min at t+19–21m; navigate respawned
  both times (startup marker logged 3×). Recovery worked — but see Finding 2.
- **Recovery is a cold replan, not a state restore:** on restart navigate reloads the persisted
  global path and resumes from current GPS. In practice this was seamless — the boat continued from
  where it was with no operator action.

## Test-plan details during the run

- **Voyage A** (`voyage_A_jdf_pacific.yaml`), over ~4.4 h the planner handled: **131 replans**
  — 77 collision-zone (dense AIS), 40 significant-wind-change, 10 path-TTL, 4 new-global-waypoint
  (initial + 2 mid-run reroutes) — with **0 PathNotFound** (it always found a path). All scripted
  wind/AIS/GPS/reroute events fired. `invalid_states.log` grew 15 KB → 66 KB (negligible).
- **Voyage B** (`voyage_B_haidagwaii_gale.yaml`): high replan rate as designed; navigate segfaulted
  at ~t+20m under the gale-driven replan storm.
- Recordings (4 topics each): A bag **50 MB**, B bag **2.1 MB**, plus per-voyage `launch.log`,
  `result.json`, and 2-min `health2_*.log` traces.

## Pathfinding performance — "is the boat going where it needs to?"

**Voyage A: yes — correct heading, steady progress, but slow (upwind).**
- Distance-to-destination fell **712.3 → ~695.0 km** (net ~17 km) over ~4.4 h → **VMG ≈ 3.9 km/h**.
- The global-waypoint target advanced **5 → 4 → 3** (real route progress), and `desired_heading`
  was continuously live, swinging ±90°+ — i.e. the boat was **tacking upwind**, which is why the
  along-route VMG is low. The boat consistently tracked NW toward the destination (short backtracks
  are individual tacks, expected on a beat).
- Because the route is ~700 km and largely upwind, the boat covered only a fraction of it in 4.4 h —
  expected, not a defect. **Observation for the team:** upwind VMG (~4 km/h) is low; if realistic
  passages are upwind-heavy, effective speed-made-good is worth a closer look (separate from #1065).
- **No stalls, no circling, no PathNotFound** — pathfinding stayed healthy the entire voyage,
  including across the respawn.

**Voyage B:** same slow-upwind character (~3.8 km/h) for its 19 min before the crash cluster;
target waypoint didn't advance in that short window.

## Fatal / uncaught crashes

**None.** All 3 navigate segfaults were caught by respawn and recovered. No other node ever died. No
new "extended" bug surfaced past the respawn (no memory blowups, no deadlocks, no pathfinding
corruption on restart).

## Findings

1. **The respawn fix resolves the #1065 mission-ending crash.** An intermittent OMPL segfault no
   longer ends the voyage — navigate auto-recovers and pathfinding continues. Demonstrated over a
   full multi-hour voyage (A).
2. **The `run_test_plans` 10 s node-missing watchdog limits high-crash-rate testing (harness issue,
   not a respawn issue).** Voyage B's extreme replan rate caused **two segfaults within ~2 min**;
   the chained crash→respawn cycles left `/navigate_main` absent from the ROS graph for >10 s, so the
   runner marked the test FAILED and tore down the whole stack — *even though each respawn succeeded*.
   On the real boat (plain `ros2 launch` with respawn, no such watchdog) the voyage would have kept
   going. **Recommendation:** for future respawn/endurance testing, raise `NODE_MISSING_TIMEOUT_SEC`
   (e.g. 30–60 s) or run the launch directly, so a burst of crashes doesn't end the run.
3. **Crashes are rare at moderate replan rates.** A saw 1 segfault in 131 replans over 4.4 h — well
   within respawn's ability to keep up. Crash frequency scales with replan intensity (consistent with
   the earlier campaign): the busier the planner, the more respawns.
4. **Reminder for operators:** because a respawn within 10 s leaves no mark in `result.json`, count
   crashes/respawns from `launch.log` (`process died exit -11` / `process started`) or the new
   `navigate_main node started (pid=…)` marker. Both worked perfectly here.

## Before vs. after the fix

| | Before (respawn off) | After (respawn on) |
|---|---|---|
| Voyage A | **died permanently @ ~4h11m** | **survived full ~4.4 h** (1 crash auto-recovered) |
| Voyage B | died permanently @ ~21m | 2 crashes auto-recovered; ended only via harness watchdog |

## Artifacts (`sim_runs/1065_voyages/`)
`RESPAWN_TEST_SUMMARY.md` (this file) · `health2_B.log`, `health2_A.log` (2-min traces) ·
`DONE2_B.txt`, `DONE2_A.txt` · `recordings2_B/`, `recordings2_A/` (result.json, launch.log, bag) ·
`supervise_respawn.sh` (harness) · `campaign2.env`. The fix under test: uncommitted changes to
`src/local_pathfinding/launch/main_launch.py` and `src/local_pathfinding/local_pathfinding/node_navigate.py`.
