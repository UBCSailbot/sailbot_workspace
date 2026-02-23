# Local Pathfinding Architecture Guide

This guide explains the local_pathfinding module architecture to newcomers by walking through what happens during the main control loop.

## The Big Picture

The local_pathfinding system runs in a **periodic loop every 0.5 seconds** (controlled by `pub_period_sec`). Each cycle:

1. **Receives** current GPS position, wind data, AIS ship positions, and target waypoint
2. **Computes** a collision-free path from current position to target waypoint
3. **Publishes** the desired heading for the boat to follow

Refer to the `architecture.puml` diagram while reading this!

## Main Components (from the Architecture Diagram)

### 1. **Sailbot** (ROS 2 Node)

- **Role**: Main orchestrator; runs the periodic control loop
- **Key methods**:
    - `desired_heading_callback()` - Called every 0.5s by a timer (the main loop)
    - `get_desired_heading()` - Orchestrates pathfinding logic
    - `publish_local_path_data()` - Publishes visualization data
- **Subscriptions**: Receives GPS, AIS ships, global path, wind data from ROS topics
- **Publications**: Sends desired heading and local path data to ROS topics

### 2. **LocalPath** (Path Manager)

- **Role**: Decides whether to compute a new path or reuse the old one
- **Key methods**:
    - `update_if_needed()` - The smart decision-maker
        - Creates new path if: global waypoint changed, no old path, or old path hits obstacle
        - Compares new path to old path using cost and heading metrics
        - Returns: `(desired_heading, waypoint_index)` back to Sailbot
    - `calculate_desired_heading_and_waypoint_index()` - Computes bearing to next waypoint
    - `in_collision_zone()` - Checks if a path crosses any obstacles
- **Creates**: `LocalPathState` (data container for current state)
- **Uses**: `OMPLPath` (the actual path planner)

### 3. **LocalPathState** (Data Container)

- **Role**: Stores all the information needed to plan a path
- **Attributes**:
    - Position, heading, speed (from GPS)
    - AIS ships, wind data
    - Target global waypoint
    - Obstacles (initialized later)
- **No methods** - just a data structure; created fresh each cycle

### 4. **OMPLPath** (Motion Planner)

- **Role**: Uses OMPL library to find collision-free paths
- **Key methods**:
    - `__init__()` - Sets up state space and solves path (max 1 second to find solution)
    - `get_path()` - Returns waypoints as lat/lon coordinates
    - `get_cost()` - Returns path cost (used to compare paths)
    - `init_obstacles()` - Extracts boat and land obstacles from the state
- **Important**: Runs collision detection via OMPL's state validity checker

### 5. **Obstacle** (Collision Zones)

- **Role**: Represents things to avoid (boats and land)
- **Subclasses**:
    - `Boat` - Dynamic obstacle from AIS ships; has collision zone around it
    - `Land` - Static obstacle; uses pre-loaded land database
- **Key methods**:
    - `is_valid()` - Checks if a point is collision-free
    - `update_collision_zone()` - Updates obstacle position/size
    - `update_sailbot_data()` - Updates with current boat position

### 6. **GlobalPath** (Mission Path)

- **Role**: Stores and interpolates the mission waypoints from GPS to destination
- **Key methods**:
    - `get_path()` - Reads waypoints from CSV file
    - `generate_path()` - Creates waypoints between two points
    - `interpolate_path()` - Adds waypoints if spacing too large
    - `calculate_interval_spacing()` - Checks waypoint distances
- **Not dynamic** - changes only when mission plan updates

## The Periodic Control Loop

See the **"Simplified Callback Sequence"** box at the bottom of `architecture.puml`. Here's what happens each cycle:

```
Timer (0.5s tick)
  ↓
Sailbot.desired_heading_callback()
  ↓
get_desired_heading()
  ↓
LocalPath.update_if_needed()
  ├─ Creates LocalPathState
  ├─ Creates OMPLPath (1 second max to solve)
  │   ├─ OMPLPath.init_obstacles()
  │   │   └─ Checks Boat and Land obstacles
  │   └─ Solves path with OMPL planner
  ├─ Calculates desired heading to next waypoint
  └─ Returns (heading, waypoint_index) ← back up the chain
  ↓
Publish to ROS "desired_heading" topic
  ↓
publish_local_path_data()
  ↓
Publish to ROS "local_path" topic
  ↓
[Wait 0.5s, repeat]
```

## Data Flow (Arrows in Diagram)

**Black arrows** = method calls (function calls, data requests)
**Red dashed arrows** = return values (data flowing back)

Key data flows to watch:

1. **Sailbot → LocalPath**: Sends GPS, AIS, global path, wind
2. **LocalPath → Sailbot**: Returns desired heading and waypoint index
3. **OMPLPath → Obstacle**: Initializes collision zones
4. **Obstacle → OMPLPath**: Returns obstacle list for path validation

## Why This Design?

- **Separation of Concerns**: Sailbot orchestrates, LocalPath decides, OMPLPath plans, Obstacle checks
- **Reusability**: Can reuse old paths if they're still valid (avoid expensive OMPL calls)
- **Extensibility**: Easy to swap planners or add new obstacle types
- **Testability**: Each component can be tested independently with mock data

## Common Questions

**Q: Why does LocalPath sometimes return the old path instead of the new one?**
A: If the old path is still collision-free and has a better cost/heading metric than the new path, we reuse it. This saves computation since OMPL solving takes time.

**Q: What happens if OMPLPath can't find a solution?**
A: It returns an empty path, and LocalPath uses the old path if available. Eventually OMPL will find a solution before hitting an obstacle.

**Q: Why are obstacles updated every cycle?**
A: AIS ships move, and our position changes, so collision zones must be recalculated. Land is static but reprojected based on our new reference waypoint.

**Q: What's the reference point?**
A: It's the next global waypoint we're heading towards. We plan in XY coordinates with this as the origin, then convert back to lat/lon.

## Recommended Reading Order

1. Read `node_navigate.py` - See Sailbot class and the callback
2. Read `local_path.py` - See LocalPath.update_if_needed() logic
3. Read `ompl_path.py` - See OMPLPath initialization and solving
4. Read `obstacles.py` - See Boat and Land obstacle logic
5. Read `global_path.py` - Understand how global waypoints are managed

Good luck! 🧭
