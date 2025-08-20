cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Add more dependencies manually with: find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Common custom messages
set(common_msg
  # external
  "msg/AISShips.msg"
  "msg/Batteries.msg"
  "msg/CanSimToBoatSim.msg"
  "msg/DesiredHeading.msg"
  "msg/GenericSensors.msg"
  "msg/GPS.msg"
  "msg/LPathData.msg"
  "msg/Path.msg"
  "msg/SailCmd.msg"
  "msg/WindSensor.msg"
  "msg/WindSensors.msg"
  "msg/TempSensor.msg"
  "msg/TempSensors.msg"
  "msg/PhSensor.msg"
  "msg/PhSensors.msg"
  "msg/SalinitySensor.msg"
  "msg/SalinitySensors.msg"
  "msg/PressureSensor.msg"
  "msg/PressureSensors.msg"

  # internal
  "msg/HelperAISShip.msg"
  "msg/HelperBattery.msg"
  "msg/HelperDimension.msg"
  "msg/HelperGenericSensor.msg"
  "msg/HelperHeading.msg"
  "msg/HelperLatLon.msg"
  "msg/HelperROT.msg"
  "msg/HelperSpeed.msg"
  "msg/HelperTemp.msg"
  "msg/HelperPh.msg"
  "msg/HelperSalinity.msg"
  "msg/HelperPressure.msg"
  "msg/HelperObstacle.msg"
)

# Boat simulator custom messages
set(simulator_msg
  # external
  "msg/SimWorldState.msg"
)

# Boat simulator custom actions
set(simulator_action
  "action/SimRudderActuation.action"
  "action/SimSailTrimTabActuation.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Add custom messages
  ${common_msg}
  ${simulator_msg}
  ${simulator_action}

  # Add packages that above messages depend on below if needed
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
