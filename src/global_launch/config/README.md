# Sailbot ROS Parameter Configuration

The description of each parameter contained in `globals.yaml` are described in
this README. Descriptions of parameters for each node are included. These
parameters can be changed dynamically as well via the command line interface.
To learn more, see the ROS 2 documentation on
<!-- markdownlint-disable-next-line MD013 -->
[ROS 2 Parameters](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html).

Each parameter is specified in the following format:

- _Description_: The description of the parameter.
- _Datatype_: The datatype. If it happens to be an array, the datatype of the
  elements should be specified and the length of the array.
- _Range_/_Acceptable Values_: Ranges of integers and floating point values
  are specified with interval notation. Namely, `[]` denotes inclusive
  boundaries, while `()` denotes non-inclusive boundaries. For strings, the
  acceptable values are listed.

Additional information may be included when necessary.

> [!IMPORTANT]
> This document should be updated when any changes occur to the ROS
> parameters specified in `globals.yaml`.

## Global Parameters

ROS parameters common across all ROS nodes in the network.

**`pub_period_sec`**

- _Description_: The period at which the publishers publish.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

## Local Pathfinding Parameters

ROS parameters specific to the nodes in the local_pathfinding package.

### `global_path`

The production node that reads the global path from a csv file and
publishes it (interpolated to `global_path_interval_spacing_km`) on the
`global_path` topic for local pathfinding to consume.

**`global_path_filepath`**

- _Description_: The absolute filepath to a global path csv file. The file
  must have a `latitude,longitude` header followed by one waypoint per row,
  ordered final destination first and launch area last (`navigate` treats
  waypoint index 0 as the final destination and heads toward it starting from
  the last waypoint).
- _Datatype_: `string`
- _Acceptable Values_: Any valid filepath to a properly formatted csv file.

**`global_path_interval_spacing_km`**

- _Description_: The upper bound on the spacing between consecutive points in
  the global path in km. The path read from the csv file is interpolated so
  that no two consecutive waypoints are farther apart than this value. This is
  a global parameter shared by all nodes.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

### `mock_global_path`

The development-only node that publishes a mock global path sourced from the
active test plan.

**`gps_threshold`**

- _Description_: A new path will be generated if the GPS position changed by
  more than `gps_threshold * global_path_interval_spacing_km`.
- _Datatype_: `double`
- _Acceptable Values_: `(1.0, MAX_DOUBLE)`

### `mock_ais`

The development-only node that publishes mock AIS data in
different modes based on whether running on water (production) or in development.

**`on_water_mock_ais`**

- _Description_: Enable or disable mock AIS data generation.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`on_water_test_plan`**

- _Description_: The test plan file to use for mock AIS data when on water.
- _Datatype_: `string`
- _Acceptable Values_: Any valid test plan filename.

### `mock_gps`

The development-only node that publishes mock GPS data with
configurable noise and drift.

**`use_gps_noise`**

- _Description_: Enable Gaussian noise on GPS readings.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`use_ocean_drift`**

- _Description_: Enable cumulative ocean current drift on GPS readings.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`use_drift_randomization`**

- _Description_: Enable small random variation to the ocean drift current each tick.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`ocean_drift_speed_kmph`**

- _Description_: Base speed of the ocean current in km/h over ground.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`ocean_drift_dir_deg`**

- _Description_: Direction the current flows toward in degrees (0=north, 90=east).
- _Datatype_: `double`
- _Range_: `(-180.0, 180.0]`

**`ocean_drift_accel_kmph2`**

- _Description_: Acceleration of the drift speed in km/h². Set to 0 for constant drift.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

### `navigate_main`

**`path_planner`**

- _Description_: The path planner used by local pathfinding. Local pathfinding currently
  uses OMPL RRT* exclusively.
- _Datatype_: `string`
- _Acceptable Values_: `"rrtstar"`

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

**`apparent_wind_lower_threshold_kmph`**

- _Description_: The lower wind threshold value for apparent wind.
- _Datatype_: 'double'
- _Range_: '[0.0, MAX_DOUBLE)'
- The value is in Kmph.

**`apparent_wind_upper_threshold_kmph`**

- _Description_: The higher wind threshold value for apparent wind.
- _Datatype_: 'double'
- _Range_: '[0.0, MAX_DOUBLE)'
- The value is in Kmph.

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

- _Description_: The maximum number of subscription messages to queue for
  further processing.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`rudder.actuation_execution_period_sec`**

- _Description_: The period at which the main loop in the rudder action
  server executes in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`rudder.disable_actuation`**

- _Description_: Controls whether or not rudder actuation is disabled. If
  true, the rudder angle is fixed to some value. Otherwise, the PID mechanism
  is used to control the rudder angle.
- Datatype: `boolean`
- _Acceptable Values_: `true`, `false`

**`rudder.fixed_angle_deg`**

- _Description_: The angle to fix the rudder in degrees. Only used if
  `rudder.disable_actuation` is true.
- _Datatype_: `double`
- _Range_: `[-45.0, 45.0]`

**`rudder.pid.buffer_size`**

- _Description_: The buffer size of PID that stores previously computed errors
  over time.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`rudder.pid.kd`**

- _Description_: The PID Derivative constant for the rudder. Only used if
  `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`rudder.pid.ki`**

- _Description_: The PID Integral constant for the rudder. Only used if
  `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`rudder.pid.kp`**

- _Description_: The PID Proportionality constant for the rudder. Only used
  if `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`rudder.pid.cp`**

- _Description_: The tuning parameter for the rudder control action. Only
  used if `rudder.disable_actuation` is false.
- _Datatype_: `double`
- _Range_: `[0.0, MAX_DOUBLE)`

**`wingsail.actuation_execution_period_sec`**

- _Description_: The period at which the main loop in the sail action server
  executes in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wingsail.actuation_speed_deg_per_sec`**

- _Description_: The speed at which the wingsail trim tab actuates in degrees
  per second.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wingsail.disable_actuation`**

- _Description_: Controls whether or not wingsail trim tab actuation is
  disabled. If true, the trim tab is fixed to some value. Otherwise, the trim
  tab angle is determined by the wingsail controller.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`wingsail.fixed_angle_degree`**

- _Description_: Fixed the wingsail trim tab to some angle in degrees. Only
  used if `wingsail.disable_actuation` is true.
- _Datatype_: `double`
- _Range_: `[-180.0, 180.0)`

### `physics_engine_node`

**`action_send_goal_timeout_sec`**

- _Description_: How long the action clients wait for the action server to
  respond to a request before timing out in seconds.
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

- _Description_: The maximum number of subscription messages to queue for
  further processing.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`rudder.actuation_request_period_sec`**

- _Description_: How often the rudder action client requests a rudder
  actuation in seconds.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wingsail.actuation_request_period_sec`**

- _Description_: How often the sail action server requests a wingsail actuation.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

**`wind_sensor.constant_params.value`**

- _Description_: Specifies the constant vector returned by the constant
  generator that represents the wind velocity in kmph. Namely, the same value
  is fixed in the wind sensors. The value is an array containing the `x` and
  `y` components of the velocity. Only used if `wind_sensor.generator_type`
  is `constant`.
- _Datatype_: `double` array, length 2
- _Range_: `(MIN_DOUBLE, MAX_DOUBLE)`

**`wind_sensor.gaussian_params.corr_xy`**

- _Description_: The correlation coefficient between x and y components of
  the wind velocity. Only used if `wind_sensor.generator_type` is `gaussian`.
- _Datatype_: `double`
- _Range_: `[-1.0, 1.0]`

**`wind_sensor.gaussian_params.mean`**

- _Description_: The mean wind velocity parameter in kmph for the gaussian
  generator. The mean is an array containing the `x` and `y` components of
  the velocity. Only used if `wind_sensor.generator_type` is `gaussian`.
- _Datatype_: `double` array, length 2
- _Range_: `(MIN_DOUBLE, MAX_DOUBLE)`

**`wind_sensor.gaussian_params.std_dev`**

- _Description_: The standard deviation parameters in kmph for the gaussian
  generator. There are two standard deviations specified within an array: one
  for the `x` component, and one for the `y` component. Only used if
  `wind_sensor.generator_type` is `gaussian`.
- _Datatype_: `double` array, length 2
- _Range_: `(0.0, MAX_DOUBLE)`
    - If a standard deviation of zero is desired, then consider using the
      constant generator instead.

**`wind_sensor.generator_type`**

- _Description_: Determines the type of random number generator that will be
  used to generate wind sensor data.
- _Datatype_: `string`
- _Acceptable Values_: `gaussian`, `constant`

**`wind_generation.mvgaussian_params.mean`**

- _Description_: The mean value for the wind generated, expressed in
  kilometers per hour (km/h), for the multivariate Gaussian generator.
- _Datatype_: `double` array, length 2
- _Range_: `(0.0, MAX_DOUBLE)`

**`wind_generation.mvgaussian_params.cov`**

- _Description_: The covariance matrix for the generated wind, represented as
  a string formatted as a 2D `double` array, since ROS parameters do not
  support native 2D array types.
- _Datatype_: `string`
- _Range_: `(0.0, MAX_DOUBLE)`

**`current_generation.mvgaussian_params.mean`**

- _Description_: The mean value for the current generated, expressed in
  kilometers per hour (km/h), for the multivariate Gaussian generator.
- _Datatype_: `double` array, length 2
- _Range_: `(0.0, MAX_DOUBLE)`

**`current_generation.mvgaussian_params.cov`**

- _Description_: The covariance matrix for the generated current, represented
  as a string formatted as a 2D `double` array, since ROS parameters do not
  support native 2D array types.
- _Datatype_: `string`
- _Range_: `(0.0, MAX_DOUBLE)`

### `data_collection_node`

**`file_name`**

- _Description_: The name of the file in which the data is saved, excluding
  the file extension.
- _Datatype_: `string`
- _Acceptable Values_: Any valid file name.

**`qos_depth`**

- _Description_: The maximum number of subscription messages to queue for
  further processing.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`topics`**

- _Description_: Specifies the topics to subscribe to. It should adhere to
  the format `['topic_name_1', 'topic_type_1', ...]`.
- _Datatype_: `string` array with an even length
- _Acceptable Values_: Each pair within the array must consist of a valid
  topic name as the first string and the corresponding correct type as the
  second string.

**`bag`**

- _Description_: Determines whether to save recorded data as a ROS bag.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`json`**

- _Description_: Determines whether to save recorded data as a JSON file.
- _Datatype_: `boolean`
- _Acceptable Values_: `true`, `false`

**`write_period_sec`**

- _Description_: The interval (in seconds) for writing queued data to the
  JSON file.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`

### `Mock Data Node`

**`qos_depth`**

- _Description_: The maximum number of subscription messages to queue for
  further processing.
- _Datatype_: `int`
- _Range_: `[1, MAX_INT)`

**`mock_desired_heading`**

- _Description_: Set to True if mock data for desired heading should be
  generated. False otherwise.
- _Datatype_: `bool`
- _Range_: `(True, False)`

**`mock_desired_heading_lower_bound`**

- _Description_: Gives a lower bound for randomizing desired heading values.
  This value should be less than `mock_desired_heading_upper_bound`.
- _Datatype_: `double`
- _Range_: `(-MAX_DOUBLE, MAX_DOUBLE)`

**`mock_desired_heading_upper_bound`**

- _Description_: Gives a upper bound for randomizing desired heading values.
  This value should be greater than `mock_desired_heading_upper_bound`.
- _Datatype_: `double`
- _Range_: `(-MAX_DOUBLE, MAX_DOUBLE)`

**`mock_sail_trim_tab`**

- _Description_: Set to True if mock data for sail trim tab should be
  generated. False otherwise.
- _Datatype_: `bool`
- _Range_: `(True, False)`

**`mock_sail_trim_tab_lower_bound`**

- _Description_: Gives a lower bound for randomizing sail trim tab values.
  This value should be less than `mock_sail_trim_tab_upper_bound`.
- _Datatype_: `double`
- _Range_: `(-MAX_DOUBLE, MAX_DOUBLE)`

**`mock_sail_trim_tab_upper_bound`**

- _Description_: Gives a upper bound for randomizing sail trim tab values.
  This value should be greater than `mock_sail_trim_tab_lower_bound`.
- _Datatype_: `double`
- _Range_: `(-MAX_DOUBLE, MAX_DOUBLE)`

**`pub_period_sec`**

- _Description_: The period at which the publishers publish.
- _Datatype_: `double`
- _Range_: `(0.0, MAX_DOUBLE)`
