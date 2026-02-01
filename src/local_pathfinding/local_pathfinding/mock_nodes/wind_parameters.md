# Setting True Wind Parameters

## Overview

The mock nodes use true wind parameters (global frame) which are converted to apparent wind (boat frame) for testing. Both `mock_wind_sensor` and `mock_gps` nodes must have matching true wind parameters.

## How to Change Parameters

### Step 1: Edit `wind_params.yaml`

Modify the constants section:

```yaml
constants:
  tw_dir_deg: &dir 0           # True wind direction (degrees)
  tw_speed_kmph: &speed 100.0  # True wind speed (kmph, must be double)
```

**Parameters:**
- `tw_dir_deg`: True wind direction in global frame
  - Valid range: (-180, 180]
  - 0° = North, 90° = East, 180° = South, -90° = West
- `tw_speed_kmph`: True wind speed in km/h
  - Must be a decimal value (e.g., 100.0 not 100)

### Step 2: Ensure Nodes are Running

Both required nodes must be active:
- `/mock_wind_sensor`
- `/mock_gps`

### Step 3: Run `wind_params.sh`

Execute the shell script to load parameters into both nodes:

```bash
./src/local_pathfinding/local_pathfinding/mock_nodes/wind_params.sh
```

## Important Warnings

**DO NOT** use `ros2 param set` directly to set `tw_speed_kmph` or `tw_dir_deg`. This will cause parameter mismatch between the two nodes and break wind calculations. Always use the `wind_params.sh` script.

## How It Works

The `mock_wind_sensor` node:
1. Takes true wind parameters in the global frame
2. Subscribes to GPS data for boat heading and speed
3. Converts true wind to apparent wind in boat frame
4. Publishes on `filtered_wind_sensor` topic