# Sailbot ROS Parameter Configuration

## Global Parameters

ROS parameters common across all ROS nodes in the network.

**`pub_period_sec`**

- _Description_: The period at which the publishers publish.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

## Boat Simulator Parameters

ROS parameters specific to the nodes in the boat simulator.

### `low_level_control_node`

**`logging_throttle_period_sec`**

- _Description_: Controls the message logging throttle period.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`info_log_throttle_period_sec`**

- _Description_: Limits the info logs to avoid overwhelming the terminal.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`qos_depth`**

- _Description_: The maximum number of subscription messages to queue for further processing.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`rudder.disable_actuation`**

- _Description_: Controls whether or not rudder actuation is disabled. If true, the rudder angle is fixed to some value.
Otherwise, the PID mechanism is used to control the rudder angle.
- Datatype: `boolean`
- _Acceptable Values_: `true`, `false`

**`rudder.fixed_angle_deg`**

- _Description_: The angle to fix the rudder in degrees. Only used if `rudder.disable_actuation` is true.
- _Datatype_: `double`
- _Range_: `[-45.0, 45.0]`

**`rudder.actuation_execution_period_sec`**

- _Description_: The period at which the main loop in the rudder action server executes in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`rudder.pid.kp`**

- _Description_: The PID Proportionality constant for the rudder. Only used if `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`rudder.pid.ki`**

- _Description_: The PID Integral constant for the rudder. Only used if `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`rudder.pid.kd`**

- _Description_: The PID Derivative constant for the rudder. Only used if `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`rudder.pid.buffer_size`**

- _Description_: The buffer size of PID that stores previously computed errors over time.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`wingsail.disable_actuation`**

- _Description_: Controls whether or not wingsail trim tab actuation is disabled. If true, the trim tab is fixed to some
value. Otherwise, the trim tab angle is determined by the wingsail controller.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`wingsail.fixed_angle_degree`**

- _Description_: Fixed the wingsail trim tab to some angle in degrees. Only used if `wingsail.disable_actuation` is true.
- _Datatype_: `double`
- _Range_: `[-180.0, 180.0)`

**`wingsail.actuation_execution_period_sec`**

- _Description_: The period at which the main loop in the sail action server executes in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wingsail.actuation_speed_deg_per_sec`**

- _Description_: The speed at which the wingsail trim tab actuates in degrees per second.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

### physics_engine_node Parameters

#### physics_engine_node `logging_throttle_period_sec`

Controls the message logging throttle period.

Datatype: `double`

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

#### physics_engine_node `info_log_throttle_period_sec`

Limits the info logs to avoid overwhelming the terminal. It's really more for us humans.

Datatype: `double`

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

#### `action_send_goal_timeout_sec`

How long the action clients wait for the server before timing out.

Datatype: `double`

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

#### `qos_depth`

The maximum number of subscription messages to queue for further processing.

Datatype: `int`

Acceptable Value Range: `[1, MAX_INT)`

#### `rudder.actuation_request_period_sec`

How often the rudder action client requests a rudder actuation.

Datatype: `double`

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

#### `wingsail.actuation_request_period_sec`

How often the sail action server requests a wingsail actuation.

Datatype: `double`

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

#### `wind_sensor.generator_type`

Datatype: `string`

Acceptable Values: `gaussian`, `constant`

#### `wind_sensor.gaussian_params.mean`

Mean for the x and y components.

Datatype: `double` array, length 2

Acceptable Value Range: `(MIN_DOUBLE, MAX_DOUBLE)`

#### `wind_sensor.gaussian_params.std_dev`

Standard deviation for the x and y components.

Datatype: `double` array, length 2

Acceptable Value Range: `[0.0, MAX_DOUBLE)`

#### `wind_sensor.gaussian_params.corr_xy`

Correlation coefficient between x and y components.

Datatype: `double`

Acceptable Value Range: `[-1.0, 1.0]`

#### `wind_sensor.constant_params.value`

Constant value for x and y components.

Acceptable Value Range: `(MIN_DOUBLE, MAX_DOUBLE)`
