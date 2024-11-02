# Sailbot ROS Parameter Configuration

The description of each parameter contained in `globals.yaml` are described in this README. Descriptions of parameters
for each node are included. These parameters can be changed dynamically as well via the command line interface. To
learn more, see the ROS 2 documentation on [ROS 2 Parameters](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html).

Each parameter is specified in the following format:

- _Description_: The description of the parameter.
- _Datatype_: The datatype. If it happens to be an array, the datatype of the elements should be specified and the length
of the array.
- _Range_/_Acceptable Values_: Ranges of integers and floating point values are specified with interval notation.
Namely, `[]` denotes inclusive boundaries, while `()` denotes non-inclusive boundaries. For strings, the acceptable
values are listed.

Additional information may be included when necessary.

> [!IMPORTANT]
> This document should be updated when any changes occur to the ROS parameters specified in `globals.yaml`.

## Global Parameters

ROS parameters common across all ROS nodes in the network.

**`pub_period_sec`**

- _Description_: The period at which the publishers publish.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

## Local Pathfinding Parameters

ROS parameters specific to the nodes in the local_pathfinding package.

### `mgp_main`

**`global_path_filepath`**

- _Description_: The absolute filepath to a global path csv file.
- _Datatype_: `string`
- _Acceptable Values_: Any valid filepath to a properly formatted csv file.

**`interval_spacing`**

- _Description_: The upper bound on spacing between each point in the global path in km.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`write`**

- _Description_: Whether or not to write a generated global path to a new csv file.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`gps_threshold`**

- _Description_: A new path will be generated if the GPS position changed by more thangps_threshold*interval_spacing.
- _Datatype_: `double`
- _Acceptable Values_: `(1.0, MAX_DOUBLE)`

**`force`**

- _Description_: Force the mock global path callback to update the global path when set to true.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

### `navigate_main`

**`path_planner`**

- _Description_: The path planner to use. Planners are from [OMPL Library](https://ompl.kavrakilab.org/planners.html).
- _Datatype_: `string`
- _Acceptable Values_: `"bitstar"`, `"bfmtstar"`, `"fmtstar"`, `"informedrrtstar"`, `"lazylbtrrt"`, `"lazyprmstar"`,
  `"lbtrrt"`, `"prmstar"`, `"rrtconnect"`, `"rrtsharp"`, `"rrtstar"`, `"rrtxstatic"`, `"sorrtstar"`

## Controller Parameters

ROS parameters specific to the nodes in the Controller.

### wingsail_ctrl_node

**`reynolds_number`**

- _Description_: The Reynolds number of the wind.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`angle_of_attack`**

- _Description_: The angle of attack of the sail.
- _Datatype_: `double`
- _Range_: `(-180.0, 180.0]`

## Boat Simulator Parameters

ROS parameters specific to the nodes in the boat simulator.

### `low_level_control_node`

**`info_log_throttle_period_sec`**

- _Description_: Limits the info logs to avoid overwhelming the terminal.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`logging_throttle_period_sec`**

- _Description_: Controls the message logging throttle period.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`qos_depth`**

- _Description_: The maximum number of subscription messages to queue for further processing.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`rudder.actuation_execution_period_sec`**

- _Description_: The period at which the main loop in the rudder action server executes in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`rudder.disable_actuation`**

- _Description_: Controls whether or not rudder actuation is disabled. If true, the rudder angle is fixed to some value.
Otherwise, the PID mechanism is used to control the rudder angle.
- Datatype: `boolean`
- _Acceptable Values_: `true`, `false`

**`rudder.fixed_angle_deg`**

- _Description_: The angle to fix the rudder in degrees. Only used if `rudder.disable_actuation` is true.
- _Datatype_: `double`
- _Range_: `[-45.0, 45.0]`

**`rudder.pid.buffer_size`**

- _Description_: The buffer size of PID that stores previously computed errors over time.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`rudder.pid.kd`**

- _Description_: The PID Derivative constant for the rudder. Only used if `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`rudder.pid.ki`**

- _Description_: The PID Integral constant for the rudder. Only used if `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`rudder.pid.kp`**

- _Description_: The PID Proportionality constant for the rudder. Only used if `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`wingsail.actuation_execution_period_sec`**

- _Description_: The period at which the main loop in the sail action server executes in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wingsail.actuation_speed_deg_per_sec`**

- _Description_: The speed at which the wingsail trim tab actuates in degrees per second.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wingsail.disable_actuation`**

- _Description_: Controls whether or not wingsail trim tab actuation is disabled. If true, the trim tab is fixed to some
value. Otherwise, the trim tab angle is determined by the wingsail controller.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`wingsail.fixed_angle_degree`**

- _Description_: Fixed the wingsail trim tab to some angle in degrees. Only used if `wingsail.disable_actuation` is true.
- _Datatype_: `double`
- _Range_: `[-180.0, 180.0)`

### `physics_engine_node`

**`action_send_goal_timeout_sec`**

- _Description_: How long the action clients wait for the action server to respond to a request before timing out in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`info_log_throttle_period_sec`**

- _Description_: Limits the info logs to avoid overwhelming the terminal.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`logging_throttle_period_sec`**

- _Description_: Controls the message logging throttle period.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`qos_depth`**

- _Description_: The maximum number of subscription messages to queue for further processing.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`rudder.actuation_request_period_sec`**

- _Description_: How often the rudder action client requests a rudder actuation in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wingsail.actuation_request_period_sec`**

- _Description_: How often the sail action server requests a wingsail actuation.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wind_sensor.constant_params.value`**

- _Description_: Specifies the constant vector returned by the constant generator that represents the wind velocity in kmph.
Namely, the same value is fixed in the wind sensors. The value is an array containing the `x` and `y` components of the
velocity. Only used if `wind_sensor.generator_type` is `constant`.
- _Datatype_: `double` array, length 2
- _Range_: `(MIN_DOUBLE, MAX_DOUBLE)`

**`wind_sensor.gaussian_params.corr_xy`**

- _Description_: The correlation coefficient between x and y components of the wind velocity. Only used if
`wind_sensor.generator_type` is `gaussian`.
- _Datatype_: `double`
- _Range_: `[-1.0, 1.0]`

**`wind_sensor.gaussian_params.mean`**

- _Description_: The mean wind velocity parameter in kmph for the gaussian generator. The mean is an array containing
the `x` and `y` components of the velocity. Only used if `wind_sensor.generator_type` is `gaussian`.
- _Datatype_: `double` array, length 2
- _Range_: `(MIN_DOUBLE, MAX_DOUBLE)`

**`wind_sensor.gaussian_params.std_dev`**

- _Description_: The standard deviation parameters in kmph for the gaussian generator. There are two standard deviations
specified within an array: one for the `x` component, and one for the `y` component. Only used if
`wind_sensor.generator_type` is `gaussian`.
- _Datatype_: `double` array, length 2
- _Range_: `(0.0, MAX_DOUBLE)`
  - If a standard deviation of zero is desired, then consider using the constant generator instead.

**`wind_sensor.generator_type`**

- _Description_: Determines the type of random number generator that will be used to generate wind sensor data.
- _Datatype_: `string`
- _Acceptable Values_: `gaussian`, `constant`

**`wind_generation.mvgaussian_params.mean`**

- _Description_: The mean value for the wind generated, expressed in kilometers per hour (km/h), for the multivariate
Gaussian generator.
- _Datatype_: `double` array, length 2
- _Range_: `(0.0, MAX_DOUBLE)`

**`wind_generation.mvgaussian_params.cov`**

- _Description_: The covariance matrix for the generated wind, represented as a string formatted as a 2D `double` array,
since ROS parameters do not support native 2D array types.
- _Datatype_: `string`
- _Range_: `(0.0, MAX_DOUBLE)`

**`current_generation.mvgaussian_params.mean`**

- _Description_: The mean value for the current generated, expressed in kilometers per hour (km/h), for the multivariate
Gaussian generator.
- _Datatype_: `double` array, length 2
- _Range_: `(0.0, MAX_DOUBLE)`

**`current_generation.mvgaussian_params.cov`**

- _Description_: The covariance matrix for the generated current, represented as a string formatted as a 2D `double`
array, since ROS parameters do not support native 2D array types.
- _Datatype_: `string`
- _Range_: `(0.0, MAX_DOUBLE)`

### `data_collection_node`

**`file_name`**

- _Description_: The name of the file in which the data is saved, excluding the file extension.
- _Datatype_: `string`
- _Acceptable Values_: Any valid file name.

**`qos_depth`**

- _Description_: The maximum number of subscription messages to queue for further processing.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`topics`**

- _Description_: Specifies the topics to subscribe to. It should adhere to the format `['topic_name_1', 'topic_type_1', ...]`.
- _Datatype_: `string` array with an even length
- _Acceptable Values_: Each pair within the array must consist of a valid topic name as the first string and the
corresponding correct type as the second string.

**`bag`**

- _Description_: Determines whether to save recorded data as a ROS bag.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`json`**

- _Description_: Determines whether to save recorded data as a JSON file.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`write_period_sec`**

- _Description_: The interval (in seconds) for writing queued data to the JSON file.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`
