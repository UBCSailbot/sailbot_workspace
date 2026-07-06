# AGENTS.md

## Repository overview

Paths in this file are relative to `src/local_pathfinding/` unless they start
with `src/`, `docs/`, or are shown in commands.

- `local_pathfinding` is a ROS 2 `ament_python` package that turns GPS, wind,
  AIS, land, and global path inputs into local waypoints and desired headings
  using OMPL-based planning.
- Main source code: `local_pathfinding/`
- Tests: `test/`
- Test plans: `test_plans/`
- Launch files: `launch/`
- Path, land, and wind data: `global_paths/`, `land/`, and `wind/`
- Documentation: `README.md`, `WindReference.md`, `global_paths/README.md`,
  `land/README.md`, `docs/current/local_pathfinding/overview.md`, and
  `docs/reference/python/conventions.md`

For package architecture, read `README.md` first, then inspect
`node_navigate.py`, `local_path.py`, `ompl_path.py`, `ompl_validity.py`,
`ompl_objectives.py`, `coord_systems.py`, and `wind_coord_systems.py` before
changing planning behavior.

## Development setup

Use the workspace-level ROS setup. Do not add package managers or vendored
dependency copies.

Run setup and focused build from the Sailbot Workspace repository root:
`/workspaces/sailbot_workspace`, the directory containing `README.md`,
`.git/`, `src/`, `docs/`, and `scripts/`. This does not mean filesystem `/`.

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
./scripts/setup.sh
./scripts/build.sh -p local_pathfinding
source install/local_setup.bash
```

`./scripts/build.sh -p local_pathfinding` installs `ompl==1.7.0` with `pip3`
if the Python OMPL binding is missing.

Install optional local pathfinding extras only when working on the path builder
or visual tests:

```bash
pip3 install -r src/local_pathfinding/requirements.txt
```

Start local pathfinding:

```bash
ros2 launch local_pathfinding main_launch.py
```

Common development launch:

```bash
ros2 launch local_pathfinding main_launch.py \
  mode:=development test_plan:=basic.yaml
```

## Build and validation

Run the narrowest relevant checks while developing.

Focused package checks:

```bash
./scripts/build.sh -p local_pathfinding
./scripts/test.sh -p local_pathfinding
```

Focused pytest checks can be run from the Sailbot Workspace repository root
after sourcing the install:

```bash
source install/local_setup.bash
python3 -m pytest src/local_pathfinding/test/test_coord_systems.py
python3 -m pytest src/local_pathfinding/test/test_local_path.py
```

Before considering a local pathfinding task complete, run:

```bash
# Format Python with the configured VS Code Black/isort-on-save setup.
# For CLI formatting when available:
python3 -m black --line-length=99 <changed-python-files>
python3 -m isort --profile=black <changed-python-files>

LINTER=flake8 LOCAL_RUN=true scripts/ament-lint.sh
LINTER=mypy LOCAL_RUN=true scripts/ament-lint.sh
LINTER=xmllint LOCAL_RUN=true scripts/ament-lint.sh
./scripts/build.sh -p local_pathfinding
./scripts/test.sh -p local_pathfinding
```

Run the full workspace validation before finishing when the change touches
shared interfaces, launch composition, or cross-package behavior:

```bash
./scripts/run-tests.sh
```

Verify behavior with a launch scenario or test plan relevant to the change:

```bash
source install/local_setup.bash
ros2 launch local_pathfinding main_launch.py \
  mode:=development test_plan:=<plan>.yaml
ros2 topic list
ros2 topic echo /desired_heading
```

For sequential test plan runs, use the installed runner documented in
`README.md` and the Test Plans Confluence page:
<https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/2996568069/Test+Plans>.

```bash
source install/local_setup.bash
ros2 run local_pathfinding run_test_plans --list
ros2 run local_pathfinding run_test_plans --tests <plan>.yaml --num_tests 1
```

If a required check cannot run, clearly report:

1. The command attempted
2. The failure
3. Whether the failure appears related to the change

## Coding goal

When coding, implement the smallest change that satisfies the task. Do not add
extra features, broad rewrites, speculative abstractions, or large helper layers
unless the task or existing design clearly requires them.

## Engineering conventions

- Follow `docs/reference/python/conventions.md`: type hinted Python,
  Google-style docstrings for non-obvious public helpers, Black line length 99,
  isort Black profile, flake8 via `.flake8`, and mypy.
- Keep ROS-specific side effects in nodes, launch files, and ROS adapters.
  Keep coordinate math, wind math, obstacle logic, OMPL validity, and objective
  logic unit-testable without spinning ROS when practical.
- Use `custom_interfaces.msg` message types at ROS boundaries. Do not create
  parallel message dictionaries or duplicate interface schemas.
- Use `coord_systems.py` and `wind_coord_systems.py` for coordinate and wind
  conversions. Do not hand-roll angle normalization, lat/lon projection,
  true-bearing conversion, or OMPL yaw conversion.
- Before changing wind math, read `WindReference.md` and keep frame names
  explicit in variable names: `_gc` for global coordinates, `_bc` for boat
  coordinates, `_deg` for degrees, `_rad` for radians, and `_kmph` for speeds.
- Preserve the package's angle and unit conventions: lat/lon for geographic
  positions, local XY distances in kilometers, speed in kilometers per hour,
  true bearings in navigation convention, and OMPL yaw in Cartesian convention
  where the OMPL helpers require it.
- Keep OMPL state validity in `ompl_validity.py`, objective/cost logic in
  `ompl_objectives.py`, path generation in `ompl_path.py`, and path lifecycle
  decisions in `local_path.py` unless a refactor is explicitly part of the
  task.
- When changing global path CSVs, follow `global_paths/README.md`: include a
  `latitude,longitude` header, at least two waypoints, and
  final-destination-first ordering.
- When changing land data, follow `land/README.md`; regenerate land pickle
  artifacts deliberately and verify the visual output when coastline behavior
  changes.
- When adding nodes, launch files, data files, or test plans, update `setup.py`
  `entry_points` or `data_files`, `package.xml` dependencies, and docs as
  needed.
- Treat runtime state and generated files carefully. Do not commit accidental
  changes to caches, logs, `__pycache__/`, `.mypy_cache/`, `.pytest_cache/`,
  `invalid_states.log`, or generated mock parameters.

## Architecture constraints

- `node_navigate.py` should orchestrate ROS subscriptions, publications, path
  state, and error handling; avoid moving planner internals into the node when
  they can live in testable helpers.
- `local_path.py` owns local path state transitions, regeneration decisions,
  wind drift comparisons, and desired-heading progression.
- Obstacle construction and updates belong in `obstacles.py`; keep Shapely
  geometry assumptions localized there.
- Visualizer changes belong in `visualizer.py` and `visualizer_assets/`; keep
  visualizer failures from breaking core navigation behavior.
- Validate untrusted inputs at boundaries: launch arguments, test plan YAML,
  global path CSVs, AIS ship data, GPS messages, wind messages, and generated
  land data.
- Preserve package installability through colcon. Avoid absolute workspace
  paths in new code; use package resources or launch substitutions where
  practical.

## Git and change scope

- Do not modify unrelated packages while working in `local_pathfinding`.
- Do not discard existing uncommitted user changes.
- Review the final diff for accidental generated data, cache, log, or
  runtime-state changes.
- Do not commit secrets, credentials, tokens, or real deployment values.

## Documentation

Update documentation when changing:

- Launch arguments or run commands in `README.md`
- Wind, coordinate, unit, or waypoint ordering conventions
- Test plan format or available scenarios
- Test plan runner usage; keep `README.md` aligned with
  `local_pathfinding/run_test_plans.py` and the Test Plans Confluence page:
  <https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/2996568069/Test+Plans>
- Land, global path, or mock data generation procedures
- ROS topics, messages, actions, or behavior visible to other packages

Avoid comments that merely restate the code. Document non-obvious math, frame
conversions, planner assumptions, and safety constraints.

## Definition of done

A task is complete when:

- The requested behavior is implemented.
- Relevant tests or test plans cover the change.
- Focused local pathfinding validation passes.
- Full workspace validation runs when practical for shared or cross-package
  changes.
- The final diff contains no unrelated generated files or runtime state.
- Documentation is updated where necessary.
- Remaining assumptions, risks, or unverified behavior are reported.

## Final response

Summarize:

- What changed
- Important implementation decisions
- Tests and validation performed
- Any remaining limitations or follow-up work
