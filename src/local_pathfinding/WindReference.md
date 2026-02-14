# Wind Conventions Reference

## Coordinate Systems

### Global Coordinate System

- Uses true bearing conventions (geographic/earth frame)
- **0° = True North**, increases clockwise till 180°, decrease counter-clockwise till -180° (exclusive)
- Range: (-180, 180] degrees
- Used for true wind direction and boat heading

### Boat Coordinate System

- Relative to the boat's orientation
- **0° = directly opposite to the boat's heading** (bow to stern direction)
- Increases clockwise till 180°, decrease counter-clockwise till -180° (exclusive) relative to the boat
- Range: (-180, 180] degrees
- Used for apparent wind direction (sensor readings)

## Wind Types

### True Wind

- Wind velocity in the global/geographic coordinate frame
- What a stationary observer on the ground would measure
- **0° = wind blowing FROM south TO north** (opposite of meteorological convention)
- Independent of boat motion

### Boat Wind

- Wind created by the boat's own motion through the air
- Direction: opposite of boat heading (`boat_heading + 180°`)
- Speed: equal to boat speed

### Apparent Wind

- Wind experienced on the moving boat
- Measured relative to the boat (boat coordinate system)
- Combination of true wind + boat-induced wind
- **Apparent Wind = True Wind + Boat Wind**

## Conversion Functions

Available in [`wind_coord_systems.py`](local_pathfinding/wind_coord_systems.py):

### Coordinate System Conversions

- `boat_to_global_coordinate(boat_heading, wind_direction)` - Convert boat-frame wind to global-frame bearing
- `global_to_boat_coordinate(boat_heading, global_wind_direction)` - Convert global-frame wind to boat-frame

### Wind Conversions

- `get_true_wind(aw_dir_deg, aw_speed_kmph, boat_heading_deg, boat_speed_kmph, ret_rad)`
  Calculate true wind from apparent wind
- `get_apparent_wind(tw_dir_deg, tw_speed_kmph, boat_heading_deg, boat_speed_kmph, ret_rad)`
  Calculate apparent wind from true wind

## Key Notes

- All angles use **meteorological convention**: wind direction indicates where wind is **coming from**, not going to
- The math computes vectors in the direction wind travels, but by convention we interpret them as source directions
- Boat coordinate 0° points to the stern (opposite of heading) following true bearing conventions
