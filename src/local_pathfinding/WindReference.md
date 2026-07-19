# Wind Conventions Reference

All wind directions in the ROS interfaces and local-pathfinding calculations use
the **flow-toward convention**: the angle points in the direction the air travels.
This differs by 180° from the meteorological/source convention, which names the
direction the wind comes from.

## Coordinate Systems

### Global Coordinate System

- Uses true bearing conventions (geographic/earth frame)
- **0° = True North**, increases clockwise till 180°, decrease
  counter-clockwise till -180° (exclusive)
- Range: (-180, 180] degrees
- Used for true-wind flow direction and boat heading

### Boat Coordinate System

- Relative to the boat's orientation
- **0° = directly opposite to the boat's heading**, so the airflow points from
  bow toward stern
- Increases clockwise till 180°, decrease counter-clockwise till -180°
  (exclusive) relative to the boat
- Range: (-180, 180] degrees
- Used for apparent-wind flow direction (sensor readings)

## Wind Types

### True Wind

- Airflow velocity in the global/geographic coordinate frame
- What a stationary observer on the ground would measure
- **0° = airflow traveling from south toward north**
- Independent of boat motion

### Boat Wind

- Wind created by the boat's own motion through the air
- Flow direction: opposite of boat heading (`boat_heading + 180°`)
- Speed: equal to boat speed

### Apparent Wind

- Airflow experienced on the moving boat
- Measured relative to the boat (boat coordinate system)
- Combination of true wind + boat-induced wind
- **Apparent Wind = True Wind + Boat Wind**

## Conversion Functions

Available in [`wind_coord_systems.py`](local_pathfinding/wind_coord_systems.py):

### Coordinate System Conversions

- `boat_to_global_coordinate(boat_heading, wind_direction)` - Convert
  a boat-frame flow direction to a global-frame flow bearing
- `global_to_boat_coordinate(boat_heading, global_wind_direction)` - Convert
  a global-frame flow bearing to a boat-frame flow direction

### Wind Conversions

<!-- markdownlint-disable-next-line MD013 -->
- `aw_gc_to_tw_gc(aw_dir_deg_gc, aw_speed_kmph, boat_heading_deg, boat_speed_kmph)`
  Calculate true wind from apparent wind
<!-- markdownlint-disable-next-line MD013 -->
- `tw_gc_to_aw_gc(tw_dir_deg, tw_speed_kmph, boat_heading_deg, boat_speed_kmph)`
  Calculate apparent wind from true wind

## Key Notes

- Every stored wind angle points **where the airflow is going**.
- In global coordinates, 0° points north and 90° points east. Global wind 0°
  therefore means airflow traveling from south toward north.
- In the `WindSensor` boat frame, 0° points to the stern and angles increase
  clockwise. Sensor wind 0° therefore means airflow traveling from bow toward
  stern.
- Convert a flow-toward bearing to a meteorological/source bearing only for an
  external interface that explicitly requires it: add 180° and normalize.
- `get_true_wind_angle` returns a signed **flow-relative** angle between boat
  heading and true airflow: 0° is flow with the heading (downwind), while 180°
  is flow against the heading (upwind).
