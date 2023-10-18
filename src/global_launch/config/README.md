# Global ROS Parameters

## Boat Simulator Parameters

### low_level_control_node Parameters

#### `logging_throttle_period_sec`

Controls the message logging throttle period.

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

#### `rudder.disable_actuation`

Controls whether or not rudder actuation is disabled

Datatype: `boolean`

Acceptable Values: `true`, `false`

#### `rudder.fixed_angle_deg`

Only used if rudder actuation is disabled.

Acceptable Value Range: ``[-45.0, 45.0]``

#### `rudder.actuation_execution_period_sec`

How often the rudder action server routine's main loop executes.

Datatype: `double`

Acceptable Values: `(0.0, MAX_DOubLE)``

#### `rudder.pid.kp`

PID Proportionality Gain

Only used if rudder actuation enabled.

Determines the strength of the controller's response to the current error.

Datatype: `double`
Acceptable Value Range: `[0.0, MAX_DOUBLE)`

#### `rudder.pid.ki`

PID Integral Gain

Only used if rudder actuation enabled.

Determines the response of the controller to past errors.

Datatype: `double`
Acceptable Value Range: `[0.0, MAX_DOUBLE)`

#### `rudder.pid.kd`

PID Derivative Gain

Only used if rudder actuation enabled.

Determines the response of the controller to the rate of change of the error.

Datatype: `double`

Acceptable Value Range: `[0.0, MAX_DOUBLE)`

#### `rudder.pid.buffer_size`

Determines buffer size of PID

Datatype: `double`

Acceptable Value Range: `[1, MAX_INT)`

#### `wingsail.disable_actuation`

Controls whether or not wingsail actuation is disabled

Datatype: `boolean`

Acceptable Values: `true`, `false`

#### `wingsail.fixed_angle_degree`

Only used if wingsail actuation is disabled.

Datatype: `double`

Acceptable Value Range:`[-180.0, 180.0)`

#### `wingsail.actuation_execution_period_sec`

How often the sail action server routine's main loop

Datatype: `double`

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

### physics_engine_node

#### `logging_throttle_period_sec`

Controls the message logging throttle period.

Datatype: `double`

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

#### `rudder.actuation_request_period_sec`

How often the rudder action client requests a rudder actuation

Datatype: `double`

Acceptable Value Range: `(0.0, MAX_DOUBLE)`

#### `wingsail.actuation_request_period_sec`

How often the sail actionaction server requests a wingsail actuation

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

Constant value for x and y components

Acceptable Value Range: `(MIN_DOUBLE, MAX_DOUBLE)`
