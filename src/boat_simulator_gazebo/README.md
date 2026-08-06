# Boat Simulator — Gazebo Backend

Gazebo physics backend for UBC Sailbot's boat simulator. It is a drop-in
replacement for `boat_simulator`'s `physics_engine_node`: the rest of the system
(low-level control, pathfinding, controller, network systems) sees the exact
same ROS topics, message types, and action interactions, but boat motion comes
from Gazebo's rigid body dynamics (buoyancy, lift/drag, wind) instead of the
in-house kinematic model.

## Status

**Migration to Harmonic in progress — not yet re-verified.** The backend was
verified end to end against Ignition Fortress, using the stock `Buoyancy` system
plus a ballast offset. It has since moved to Gazebo Harmonic with flotation from
`asv_wave_sim`'s `Hydrodynamics` plugin. Both models have been re-checked
qualitatively under Harmonic — each floats at its waterline, holds attitude, and
sails under ambient wind, headless and in the GUI — but the numbers below are
still the Fortress baseline and need rerunning:

| Check                     | Result (Fortress baseline)                      |
| ------------------------- | ----------------------------------------------- |
| Flotation                 | settled at z ≈ 0.205 m (predicted 0.200 m)      |
| Attitude                  | upright, tilt converging to ≈ 2°                |
| Boat speed                | 6.6 km/h running downwind in a 7.07 km/h wind   |
| `rudder_joint` tracking   | 0.5 rad commanded → 28.66° actual (0.5 rad)     |
| `trim_tab_joint` tracking | 0.3 rad commanded → 17.18° actual (0.3 rad)     |
| `/gps`                    | resolved to Vancouver from the test-plan origin |

They were measured against
[`models/polaris_basic`](models/polaris_basic/model.sdf), the default
`boat_model_sdf`: a coarse box-geometry stand-in satisfying the [model
contract](#boat-model-contract), with hand-fitted lift/drag coefficients and
joint gains that are still worth tuning.
[`models/polaris`](models/polaris/model.sdf) is the same boat drawn with the real
CAD hull, identical in every respect except `<visual>` geometry.

## Setup

This backend targets **Gazebo Harmonic** (`gz-sim` 8, CLI `gz sim`), paired with
the `ros-humble-ros-gzharmonic` vendor packages. Harmonic is the current LTS and
is what `asv_wave_sim` needs; the Dev Container originally shipped Ignition
Fortress, which could not run the wave field at all.

Gazebo is **opt-in at image build time**, because its dependency closure is
about 1.8 GB and most of the team never runs the simulator. Enable it by
uncommenting the Gazebo compose overlay in
[`devcontainer.json`](../../.devcontainer/devcontainer.json) and rebuilding the
container; see
[`.devcontainer/gazebo/README.md`](../../.devcontainer/gazebo/README.md) for the
details and the Apple Silicon caveat.

The overlay sets `INSTALL_GAZEBO=true`, which swaps the base image over to
`ghcr.io/ubcsailbot/sailbot_workspace/gazebo-dev`. That image already contains:

1. The OSRF apt repository (Harmonic is not in the ROS one) plus `gz-harmonic`,
   `ros-humble-ros-gzharmonic`, and `libcgal-dev` / `libfftw3-dev`. Note that
   `ros-humble-ros-gzharmonic-bridge` **conflicts with** and replaces
   `ros-humble-ros-gz-bridge`, so this removes the Fortress bridge.
2. `asv_wave_sim`, built and installed to `/opt/asv_wave_sim`, with
   `ASV_WAVE_SIM_PREFIX`, `GZ_SIM_SYSTEM_PLUGIN_PATH`,
   `GZ_RENDERING_PLUGIN_PATH`, and `GZ_SIM_RESOURCE_PATH` exported to point at
   it. `GZ_RENDERING_PLUGIN_PATH` looks redundant with
   `GZ_SIM_SYSTEM_PLUGIN_PATH` but is not: the ocean surface mesh
   (`WavesVisual`) is drawn by a render-engine extension that only the GUI
   client loads, through a loader with no search path of its own. Without it the
   GUI logs `Failed to load plugin [gz-waves1-rendering-ogre2] : couldn't load
   library on path []` and the boat spawns but never renders.

Prebuilding `asv_wave_sim` into that image is deliberate: it is a pinned
dependency that never changes, and its CGAL-heavy translation units take **~15
minutes** to compile. Its system plugins have no CMake install rules — they are
meant to be used from the build tree — so the image copies the built `lib`
directory out wholesale.

Then the normal workspace setup:

```bash
export ENABLE_GAZEBO=true
./scripts/setup.sh    # resolves the conditional Gazebo deps in package.xml;
                      # skips the asv_wave_sim source import when the image has it
./scripts/build.sh
```

**`GZ_VERSION` must be `harmonic`.** `asv_wave_sim` picks its Gazebo library
versions from that variable and silently falls back to Garden's (`gz-math7` and
friends) when it is unset, which will not configure against Harmonic. Both the
Dockerfile and
[`devcontainer.json`](../../.devcontainer/devcontainer.json) set it, so a
rebuilt container gets it automatically.

<details>
<summary>Building asv_wave_sim in the workspace instead</summary>

Only needed to modify the wave physics, or on an image built without
`INSTALL_GAZEBO`. `scripts/setup.sh` falls back to importing
[`dependencies.repos`](dependencies.repos) into `src/` when
`/opt/asv_wave_sim` is absent, and `gazebo_launch.py` prefers the image prefix
but falls back to `src/asv_wave_sim`. Expect the ~15-minute build, and export
`GZ_VERSION=harmonic` first.

</details>

`ENABLE_GAZEBO` gates both the `vcs import` of
[`dependencies.repos`](dependencies.repos) in `scripts/setup.sh` and the
conditional Gazebo dependencies in [`package.xml`](package.xml), so non-Gazebo
environments are unaffected.

Those dependencies use custom rosdep keys defined in
[`custom-rosdep.yaml`](custom-rosdep.yaml) rather than the stock `ros_gz_bridge`
and `ros_gz_sim` keys. The stock keys resolve to `ros-humble-ros-gz-*`, the
Fortress builds, which `Conflict` with the Harmonic packages — so depending on
them would make `rosdep install` quietly uninstall Harmonic and restore
Fortress, breaking the backend on the next run.

**Open a new terminal after building this package for the first time**, or run
`source install/local_setup.bash` in the existing one. A shell started before the
package existed fails with:

```text
importlib.metadata.PackageNotFoundError: No package metadata was found for boat-simulator-gazebo
```

`~/.bashrc` sources the workspace overlay once at startup, so `PYTHONPATH` is a
snapshot from that moment and `--symlink-install` leaves the package metadata
somewhere it no longer points. The error is misleading — the executable itself is
found through the ament index, only its metadata is not — and older packages keep
working in the same shell, which makes the failure look specific to this one.

`asv_wave_sim` is required, not optional, since it supplies flotation. If
`gz-cmake3` is missing, `scripts/setup.sh` drops a `COLCON_IGNORE` into it so its
configure failure does not abort the whole workspace build. The world then loads
but the boat does not float, so treat "gz-cmake3 not found" as a hard failure.

## Usage

```bash
# Full simulator with the Gazebo backend instead of the kinematic physics engine:
ros2 launch boat_simulator main_launch.py physics_backend:=gazebo_gui
ros2 launch boat_simulator main_launch.py physics_backend:=gazebo_headless

# Whole system in sim mode, same argument:
ros2 launch global_launch main_launch.py \
  mode:=sim physics_backend:=gazebo_headless

# Gazebo backend alone:
ros2 launch boat_simulator_gazebo gazebo_launch.py
```

`physics_backend` selects the backend and, for Gazebo, whether the GUI runs:
`gazebo_gui` opens the Gazebo window, `gazebo_headless` runs the server only.
Both map onto this package's `gz_headless` argument below, which is what the
standalone launch file takes; when the backend is selected through
`boat_simulator`, `physics_backend` wins over any `gz_headless` passed
alongside it.

Launch arguments (in addition to the global ones):

| Argument            | Default                          | Description                                     |
| ------------------- | -------------------------------- | ----------------------------------------------- |
| `gz_world`          | `worlds/sailing_world.sdf`       | World SDF to load                               |
| `gz_headless`       | `false`                          | Server-only (no GUI); set by `physics_backend`  |
| `boat_model_sdf`    | `models/polaris_basic/model.sdf` | Boat model to spawn; empty skips spawning       |
| `boat_spawn_height` | `0.2`                            | Model origin height above mean water at spawn   |

`boat_spawn_height` is the spawned model's flotation equilibrium, so it belongs
to the model rather than the launch. The default matches `polaris_basic`; spawn
`models/polaris` with `boat_spawn_height:=0.0`:

```bash
ros2 launch boat_simulator_gazebo gazebo_launch.py \
  boat_model_sdf:=$PWD/src/boat_simulator_gazebo/models/polaris/model.sdf \
  boat_spawn_height:=0.0
```

Set `gz_headless:=true` (or `physics_backend:=gazebo_headless`, when launching
through `boat_simulator`) if there is no display. Note that the Dev Container
sets `LIBGL_ALWAYS_SOFTWARE=1` and exposes no GPU, so the GUI is rasterised on
the CPU on every host platform — it runs, but it costs about 1 GB of RSS and
holds a real-time factor near 0.8 rather than 1.0.

## Architecture

```
                     ┌──────────────────── Gazebo (Harmonic, gz-sim 8) ─────────────────────┐
                     │ sailing_world.sdf: WavesModel (asv_wave_sim), WindEffects, <wind>    │
                     │ polaris: OdometryPublisher, JointPositionController, LiftDrag, Hydro │
                     └──────────────┬──────────────────────────────▲────────────────────────┘
                                    │ /model/polaris/odometry      │ /model/polaris/joint/*/cmd_pos
                       ros_gz_bridge (BRIDGED_TOPICS in gazebo_launch.py)
                                    │ /gazebo/odometry             │ /gazebo/*_cmd
                     ┌──────────────▼──────────────────────────────┴────────────────────────┐
                     │                   gazebo_physics_bridge_node                         │
                     └──────────────┬──────────────────────────────▲────────────────────────┘
              gps, kinematics,      │                              │ desired_heading, sail_cmd,
              filtered_wind_sensor, │                              │ rudder/sail actuation actions
              rudder                ▼                              │ (served by low_level_control_node)
                              rest of the system ──────────────────┘
```

`gazebo_physics_bridge_node` mirrors `physics_engine_node`'s contract:

- **Publishes** `gps` (GPS), `filtered_wind_sensor` (WindSensor), `kinematics`
  (SimWorldState), `rudder` (HelperHeading).
- **Subscribes** to `desired_heading` (DesiredHeading) and `sail_cmd` (SailCmd).
- **Action client** of `rudder_actuation` and `sail_trim_tab_actuation` (served
  by `low_level_control_node`, which is launched in both backends); actuation
  *feedback* angles are forwarded to the Gazebo joint controllers.
- GPS latitude/longitude is derived from the Gazebo ENU position and the same
  test-plan GPS origin used by the kinematic backend.
- The apparent wind is computed from the constant world wind
  (`true_wind_enu_mps` parameter in `src/global_launch/config/globals.yaml`,
  which **must match** the `<wind>` vector in the world SDF) and the boat
  velocity. No sensor noise or filtering is applied.
- Ocean current is reported as zero (the world has no current model).

## Boat model contract

The boat is spawned by `gazebo_launch.py` from the `boat_model_sdf` launch
argument. Any SDF (or URDF converted to SDF) model works as long as it satisfies
this contract:

### Naming and frames

- The model is spawned with the name **`polaris`** (the bridged topic names and
  the Hydrodynamics `<enable>` filter depend on it).
- World frame is ENU: x east, y north, z up; the mean water level is `z = 0`,
  with the wave field displacing the actual surface about it.
- Body frame: x forward (bow), y port, z up. Yaw = 0 means the bow points east.

### Required joints

| Joint name      | Type     | Axis | Positive direction | Range                       |
| --------------- | -------- | ---- | ------------------ | --------------------------- |
| `rudder_joint`  | revolute | +Z   | CCW from above     | ±30° (±0.524 rad)           |
| `trim_tab_joint`| revolute | +Z   | CCW from above     | ±40° (±0.698 rad)           |

The `SimRudderActuation` feedback angle increases **CW**, so the bridge node
negates it before commanding `rudder_joint`; `SimSailTrimTabActuation` feedback
increases CCW and is forwarded as-is. Commands are in **radians**.

These are the contract's ROS-facing identifiers, not necessarily the SDF joint
names: what matters is that a `JointPositionController`'s `<topic>` matches
`/model/polaris/joint/<name>/cmd_pos`. Both shipped models name their SDF joints
the same, but a model that did not would still comply.

### Required plugins on the model

```xml
<!-- Pose/velocity feedback consumed by gazebo_physics_bridge_node. -->
<plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
  <dimensions>3</dimensions>
  <odom_publish_frequency>10</odom_publish_frequency>
  <!-- Publishes on /model/polaris/odometry (bridged to /gazebo/odometry). -->
</plugin>

<!-- One per actuated joint; listens on /model/polaris/joint/<joint>/cmd_pos. -->
<plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
  <joint_name>rudder_joint</joint_name>
  <topic>/model/polaris/joint/rudder_joint/cmd_pos</topic>
</plugin>
<plugin filename="gz-sim-joint-position-controller-system"
        name="gz::sim::systems::JointPositionController">
  <joint_name>trim_tab_joint</joint_name>
  <topic>/model/polaris/joint/trim_tab_joint/cmd_pos</topic>
</plugin>

<!-- Aerodynamics/hydrodynamics: one LiftDrag system per foil. Coefficient tables to
     match live in boat_simulator/common/constants.py (BOAT_PROPERTIES): sail 2.01 m²,
     rudder 0.117 m², keel 0.51 m². The sail link needs <enable_wind>true</enable_wind>
     so the world's WindEffects/wind vector reaches it. -->
<plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
  <link_name>wing_sail</link_name>
  <air_density>1.225</air_density>
  <area>2.01</area>
  <!-- cla/cda/stall parameters approximating the BOAT_PROPERTIES tables -->
</plugin>
<!-- rudder + keel LiftDrag systems use <air_density>1027</air_density> (seawater). -->
```

### Flotation (Hydrodynamics)

Flotation comes from `asv_wave_sim`'s `Hydrodynamics` plugin on the model, not
from the stock gz-sim `Buoyancy` system, which this world deliberately does not
load. The plugin meshes the link's **collision geometry** and integrates
pressure over whichever part is below the wave surface each step, so the hull
collision volume must displace ~0.269 m³ (`mass / 1027`, with mass = 276 kg
from `BOAT_PROPERTIES`) at the intended waterline. Give the hull link realistic
mass and inertia (`I_xx = 175.86`, `I_zz = 119.04` kg·m² about the CG).

```xml
<plugin filename="gz-waves1-hydrodynamics-system"
        name="gz::sim::systems::Hydrodynamics">
  <enable>polaris::hull</enable>   <!-- fully-qualified model::link -->
  <wave><!-- must match the world's WavesModel block --></wave>
  <hydrodynamics><!-- damping and drag coefficients --></hydrodynamics>
</plugin>
```

Two things to get right:

- **The `<wave>` block must match the world's `WavesModel` block exactly.** The
  two systems evaluate the wave field independently from their own parameters,
  so a mismatch floats the boat against a different surface than the one being
  rendered.
- **A ballast offset IS needed, and removing it capsizes the boat.** Both models
  put the hull's inertial `<pose>` 0.35 m below the link origin. This was
  removed once, on the reasoning that `Hydrodynamics` integrates over the
  submerged volume and so recovers a genuine waterplane righting moment — unlike
  the stock `Buoyancy` system, which applies a force that never shifts as the
  hull heels. The waterplane moment is real, but **it is not a substitute for
  ballast.** Measured with the CG on the origin, `polaris_basic` rolls past 90°
  at ~36 s and then floats fully inverted, indefinitely.

  The reason is that form stability alone leaves a hull with *two* stable
  equilibria — a box is equally happy upside down — so any disturbance past its
  range of stability leaves it there. Ballast makes upright the only equilibrium:
  with the CG below the centre of buoyancy, an inverted hull has nowhere stable
  to sit and rights itself.

- **Check `GM` when changing hull geometry, mass, or the ballast offset:**

  ```
  GM = KB + BM - KG,   BM = I_waterplane / displaced volume
  ```

  Both models land near GM ≈ 1.2–1.4 m and hold roll inside about ±15° under
  ambient wind. Note that a box's waterplane second moment (0.291 m⁴) is nearly
  5× the real hull's (0.061 m⁴), so the collision proxy is contributing much more
  form stability than the real boat has — the ballast offset is carrying the
  rest. Neither figure is a measured property of Polaris.

## Example models

Two models ship with this package; `gazebo_launch.py`'s `boat_model_sdf` picks
between them (or points at something else entirely).

**They are the same boat.** Links, joints, mass, inertia, joint limits, gains,
and every `LiftDrag`/`Hydrodynamics` parameter are identical and sourced from
`BOAT_PROPERTIES`, so a physical number changed in one must change in the other.
They differ only in `<visual>` geometry and in the vertical placement of the
hull collision box: both use the same 3.6 × 0.99 × 0.55 m buoyancy proxy, but
`polaris` sits it higher on the CG so the waterline lands where the real hull
mesh floats. Hence the different `boat_spawn_height` (0.0 vs 0.2) and GM (1.04 m
vs 0.845 m).

### `models/polaris` (real hull geometry)

[`models/polaris/model.sdf`](models/polaris/model.sdf) draws the boat with the
CAD meshes in [`models/polaris/meshes`](models/polaris/meshes), prepared from
the SolidWorks-exported URDF at
[`models/polaris/urdf/polaris.urdf`](models/polaris/urdf/polaris.urdf) by
[`tools/prepare_polaris_meshes.py`](tools/prepare_polaris_meshes.py), which
reframes the z-down export, moves the hull origin onto the CG, and decimates
`base_hull.STL` from 1.35 M triangles to ~36 k. Rerun it after replacing a CAD
export:

```bash
pip install fast-simplification
python3 src/boat_simulator_gazebo/tools/prepare_polaris_meshes.py
```

It reads the gitignored `models/polaris/meshes/raw/*.STL` and writes
`models/polaris/meshes/`; only the prepared meshes are committed, which is why
cloning does not cost 70 MB.

Two things the CAD export gets you into, both documented at the point of use in
`model.sdf`:

- **`base_hull.STL` is a thin-walled shell** (~10 mm wall), so it encloses the
  volume of the hull *skin*, not the hull's displacement — 0.177 m³ against the
  0.269 m³ needed to float. It can never be `Hydrodynamics` collision geometry
  at any triangle count; see the comment on `hull_collision`.
- **`link_wingsail.STL` has no sail skin.** It is the structural frame only: a
  mast tube and two tab booms, 1.99 m² of surface for what should be a 2.01 m²
  wing needing ~4 m² of skin on both sides. Rendered as exported, the boat
  carries a bare mast and no sail. The skin is most likely a surface body or a
  suppressed component, which the SolidWorks URDF exporter skips because it only
  walks solid bodies.

### `models/polaris_basic` (box stand-in, contract-verified)

[`models/polaris_basic/model.sdf`](models/polaris_basic/model.sdf) is the
model the [Status](#status) table above was measured against, and is a
working implementation of the contract above. It is box geometry, not hull
shape, but its physical quantities come from `BOAT_PROPERTIES` so both
backends describe the same boat:

| Link        | Geometry (m)       | Source                                |
| ----------- | ------------------ | ------------------------------------- |
| `hull`      | 3.6 × 0.99 × 0.55  | 276 kg, `I_xx` 175.86, `I_zz` 119.04  |
| `wing_sail` | 1.5 × 0.12 × 2.7   | 2.01 m², CE `SAIL_CE_REL_TO_CG`       |
| `trim_tab`  | 0.3 × 0.05 × 0.66  | 0.198 m², boom length 1.43 m          |
| `rudder`    | 0.22 × 0.04 × 0.53 | 0.117 m², CE `RUDDER_CE_REL_TO_CG`    |

The keel is part of the `hull` link (it is rigidly fixed), with its `LiftDrag`
`<cp>` set to `KEEL_CE_REL_TO_CG` so its side force acts at the right point.

Two details worth knowing before editing it:

- **The mast is free, not actuated.** `mast_joint` lets the wing weathervane
  about +Z with damping; the actuated `trim_tab_joint` is what steers the wing,
  as on the real boat. Only `rudder_joint` and `trim_tab_joint` are contract
  joints.
- **Spawn height is tied to hull geometry.** The hull box must submerge 0.075 m
  to displace 0.269 m³, so the boat spawns with its CG at
  `boat_spawn_height = 0.2` m to start at equilibrium. That default is
  `polaris_basic`'s; `models/polaris` reaches equilibrium at 0.0. Change either
  model's hull collision geometry and its spawn height must change too.

## Topic map (ros_gz_bridge)

Declared as `BRIDGED_TOPICS` in
[`launch/gazebo_launch.py`](launch/gazebo_launch.py). The ROS-side names are
produced by remapping, because `parameter_bridge` otherwise names each ROS
topic after its Gazebo counterpart.

| ROS topic                         | Gazebo topic                                  | Types (ROS / Gazebo)                  | Direction |
| --------------------------------- | --------------------------------------------- | ------------------------------------- | --------- |
| `/gazebo/clock`                   | `/clock`                                      | rosgraph_msgs/Clock / gz.msgs.Clock   | GZ → ROS  |
| `/gazebo/odometry`                | `/model/polaris/odometry`                     | nav_msgs/Odometry / gz.msgs.Odometry  | GZ → ROS  |
| `/gazebo/rudder_angle_cmd`        | `/model/polaris/joint/rudder_joint/cmd_pos`   | std_msgs/Float64 / gz.msgs.Double     | ROS → GZ  |
| `/gazebo/sail_trim_tab_angle_cmd` | `/model/polaris/joint/trim_tab_joint/cmd_pos` | std_msgs/Float64 / gz.msgs.Double     | ROS → GZ  |

## Known limitations / follow-up work

- **Roll stability is tuned, not measured.** It comes from a 0.35 m ballast
  offset plus a collision box with ~5× the real hull's waterplane inertia, and
  neither number is derived from Polaris (see
  [Flotation](#flotation-hydrodynamics)). Absolute heel angles, roll period, and
  any stability-limited manoeuvre should not be trusted.
- **The ballast offset breaks the CG convention.** The `*_CE_REL_TO_CG`
  constants are measured from the CG, but with the offset applied the link
  origin they are expressed against sits 0.35 m above the true CG. Fixing this
  properly means either putting real mass in a separate keel link or
  re-referencing the constants — both of which touch the kinematic backend too.
- **`models/polaris`'s hull mesh floats at the right draft, but for the wrong
  reason.** The box's placement was chosen to put the waterline where the mesh
  looks right, not derived from the mesh's displaced volume.
- **The wing sail's planform is a stand-in, not CAD geometry**, because the
  export contains no sail skin. The area is right by construction, but the taper
  ratios (0.85 and 0.50) and the one-third/two-thirds span split are plausible
  wingsail proportions rather than measured ones, and the span itself is
  uncertain: the frame's spars sit 0.68 m apart, which would imply a much
  shorter wing than `sail_areas / WING_SAIL_CHORD` = 2.01 m gives. Resolved by a
  re-export, not by tuning.
- **`models/polaris` reuses `polaris_basic`'s hand-fitted `LiftDrag`
  coefficients**, which are not fitted to its actual foil geometry. That is a
  deliberate consequence of keeping the two models physically identical.
- The CAD rakes the rudder stock about 21° off vertical; the model holds the
  `rudder_joint` axis vertical instead, because the contract specifies +Z and a
  raked axis would make commanded and achieved steering angles differ by
  `cos(rake)`.
- The `<wave>` parameters are duplicated between the world's `WavesModel` and
  the model's `Hydrodynamics` plugin, and nothing enforces that they agree. A
  silent mismatch floats the boat against a surface other than the rendered one.
- The `Hydrodynamics` damping and drag coefficients are asv_wave_sim's reference
  values, not values derived from `BOAT_PROPERTIES`. The kinematic backend's
  `D = diag(45, 180, 115, 140)` is in N·s/m while these are dimensionless
  coefficients scaled by submerged area, so they cannot be transcribed directly
  and the two backends will not damp identically.
- `polaris_basic` uses box geometry with hand-estimated `LiftDrag`
  coefficients fitted to the `BOAT_PROPERTIES` tables at a single point, rather
  than the full angle-of-attack curves. The kinematic and Gazebo backends will
  not agree quantitatively.
- `I_yy` and the mast/joint damping values in `polaris_basic` are guesses;
  the 4-DOF kinematic model has no pitch DOF to source `I_yy` from.
- Sailing has only been checked running downwind from a standing start. Upwind
  and reaching behaviour, and whether the boat can actually hold a commanded
  heading under the low-level controller, are untested.
- ROS nodes run on wall clock; `/gazebo/clock` is bridged but `use_sim_time` is
  not set. Fine while the world runs at real-time factor 1.0.
- Constant wind only, and no ocean current. The wave field is tuned to the same
  1.96 m/s wind, so the sea is nearly flat by default; raise `wind_speed` in
  both `<wave>` blocks to test the boat in a rougher sea.
- The `data_collection_node`, `mock_data_node`, and `sim_visualizer_node` flags
  of `boat_simulator` work unchanged with this backend.
