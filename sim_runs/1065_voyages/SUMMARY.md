# PATH Complex Scenario Verification (#1065) — Voyage Campaign Summary

**Author run:** 2026-08-16 21:02Z → 2026-08-17 05:02Z (8 h wall-clock budget)
**Module under test:** `local_pathfinding` on branch `AMaharaj16/test-scenario-1065`
(main + PR #1005 dynamic test plans) · **Mode:** development · **ROS_DOMAIN_ID:** 51
**All work is local — no commits, no pushes.**

## TL;DR

Three genuinely complex dynamic test plans were authored and run back-to-back over 8 hours.
**Two of three crashed `navigate_main` with the same fault: a compiled-OMPL RRT\* segmentation
fault (exit −11) during a local-path replan.** The crash is **intermittent and driven by
replan intensity** — the more frequently the planner replans (from aggressive wind shifts,
dense/close traffic, reroutes, drift), the sooner it segfaults. A realistic low-replan
control passage survived 3 hours untouched. **This is a launch-critical finding** for the
Tuesday deployment: sustained heavy replanning can kill the pathfinding node mid-voyage with
no recovery.

## The three voyages

| # | Scenario | Duration | Result | Replan rate | Crash trigger |
|---|---|---|---|---|---|
| **A** | Juan de Fuca → open Pacific. Dense evolving AIS (to 7-ship clusters), moderate wind (16 shifts), 2 reroutes, ocean drift. ~750 km. | **4h 11m** | **FAILED** (SIGSEGV) | 0.41/min | OMPL solve on a **collision-zone** replan, amid a 7-ship cluster + a reroute |
| **B** | Haida Gwaii gale. 20 wind shifts (gusts to 55 km/h), 8 reroutes, strong drift, light AIS. Upwind beat, ~430 km. | **21 m** | **FAILED** (SIGSEGV) | 2.52/min | OMPL solve on a **wind-change** replan (10.9° > 10° threshold) |
| **C** | Realistic westbound reach. Steady 25 km/h wind, 6 gentle shifts, distant traffic, no reroutes, mild current. ~450 km. | **2h 57m** | **TIMEOUT — survived** ✅ | 0.10/min | — (control) |

Both crashes: `navigate_main` **exit code −11 (SIGSEGV)**, **0 PathNotFound** in every run —
the planner *always found a path*, then died inside the solver, not from infeasibility.

## Key finding: crash likelihood scales with replan rate

The three voyages line up monotonically:

```
replan rate   outcome
  2.52/min  -> crash at 21 min      (Voyage B, gale + reroutes + drift)
  0.41/min  -> crash at 4h 11m      (Voyage A, dense AIS)
  0.10/min  -> no crash in 3 h      (Voyage C, realistic)
```

The fault is **not** tied to a specific trigger (it fired on a collision-zone replan in A and
a wind-change replan in B) and **not** a fixed replan count (A survived 102 local-path
switches; B died at 53). It behaves like a **probabilistic segfault per OMPL `solve()` call**:
each replan is a roll of the dice, so a higher replan rate reaches the fault sooner. Realistic
conditions (C) replan rarely and stayed up for 3 h — substantially safer, though **not proven
immune** (A crashed at 4h11m at only 0.41/min, so a long realistic voyage could still hit it).

## Root cause & code context

- The crash is inside the **compiled OMPL** `og.RRTstar` `solve()` (`ompl_path.py`), invoked on
  every replan from `LocalPath.update_if_needed()` (`local_path.py`). Exit −11 = a C++-level
  segfault, so it is **not fixable in the package's Python** — only mitigable.
- **Aggravators in current config** (all in `ompl_path.py`): `MAX_SOLVER_RUN_TIME_SEC = 1.0`
  (tight budget → RRT\* runs hot), `MIN_TURNING_RADIUS_KM = 0.05` (a 50 m Dubins turn radius →
  very deep search trees), `setRange(MAX_EDGE_LEN_KM)` is **commented out / disabled** (long,
  sparsely-collision-checked edges), and the search box grows with boat-to-goal distance
  (`BOX_BUFFER_SIZE_KM = 2.0`).
- **Replan triggers** (`local_path.py`): significant wind change (`WIND_DIRECTION_CHANGE_THRESH_DEG
  = 10`), path intersects a collision zone, path TTL (`PATH_TTL_SEC = 600`), new global waypoint,
  segment deviation. Aggressive scenarios hit these constantly.

### Recommended before launch (config-only, no code changes to the crash path)
1. **Reduce replan frequency** so the node accumulates far fewer OMPL solves per hour: raise the
   wind-change threshold, lengthen the path TTL, and/or damp reaction to transient AIS/drift.
2. **Reduce per-solve load / crash surface**: re-enable a sane `setRange`, consider a larger
   turning radius, and keep the search box bounded.
3. Treat a `navigate_main` death as expected-possible and add **automatic respawn** (the launch
   does not currently restart it) so a mid-voyage segfault is recoverable rather than terminal.
4. A proper fix requires the OMPL/compiled side (out of scope for this Python package).

## Secondary findings (verified during authoring/setup)

- **`invalid_states.log` unbounded growth + crash vector** (`ompl_path.py`, ~line 585): every
  invalid OMPL state is appended to a hardcoded `/workspaces/.../invalid_states.log`, and a failed
  write **re-raises `OSError`** out through the solver → would kill `navigate_main`. It did **not**
  fire here (949 GB free; grew only KB/hr), but the code itself carries a TODO *"remove before final
  launch to avoid unbounded disk growth."* **Recommend removing/guarding it before Tuesday.**
- **AIS test-plan parser landmine:** the parser validates a `float()`-cast copy but constructs the
  message from the **raw** YAML scalar, so AIS float fields written as integers (`speed: 20`) crash
  at load, and `rot` must be an int in `[-128,127]` (the range validator wrongly allows ±720). The
  `heading: 360.0` COG-unavailable sentinel is accepted in the initial `ais:` block but **rejected
  in AIS events** (`_check_heading`). These bit during authoring and are documented so future plan
  authors avoid them.
- **Static:** the mock `/global_path` one-shot publish is VOLATILE on both sides — a late-matching
  `navigate` subscriber can miss it (it self-rescues from a persisted CSV). Not exercised as a crash
  here.

## Method / reproducibility

- Runner: `ros2 run local_pathfinding run_test_plans -t <plan> -n 1 --timeout_hours <N> --mode
  development --log_level info --save_path <dir>` (launches with `record:=true`; auto-records
  `/gps /local_path /filtered_wind_sensor /ais_ships`). It flags a run `FAILED` when
  `navigate_main` is absent > 10 s — that is how each crash was detected.
- A bash supervisor (`supervise_voyage.sh`) health-logged every 5 min and woke the operator only on
  crash/stall/finish (token-frugal monitoring per the task guidance).
- **To reproduce a crash quickly:** run Voyage B — it segfaults in ~20 min. To reproduce the slow,
  realistic-traffic crash, run Voyage A (~4 h).

## Artifact index (`sim_runs/1065_voyages/`)

| File | What |
|---|---|
| `voyage_A_jdf_pacific.yaml` / `report_A.md` | Voyage A plan + write-up |
| `voyage_B_haidagwaii_gale.yaml` / `report_B.md` | Voyage B plan + write-up |
| `voyage_C_realistic_reach.yaml` / `report_C.md` | Voyage C plan + write-up |
| `recordings_A|B|C/` | per-run `result.json`, `launch.log`, rosbag (`datarecording_*`) |
| `health_A|B|C.log` | 5-min supervisor health traces |
| `campaign.env`, `supervise_voyage.sh` | campaign clock + supervisor harness |

The three plan YAMLs also live in `src/local_pathfinding/test_plans/` (required for the runner).
