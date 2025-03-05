#pragma once

#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/wind_sensors.hpp>
#include <string>

/**
 * ROS argument value for system mode. An enum would be a better way of representing a binary choice between the two
 * options, but since strings are not integral types they cannot be made into enums.
 */
namespace SYSTEM_MODE
{
static const std::string PROD = "production";
static const std::string DEV  = "development";
};  // namespace SYSTEM_MODE

constexpr unsigned int MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES = 340;
constexpr unsigned int MAX_REMOTE_TO_LOCAL_PAYLOAD_SIZE_BYTES = 270;

inline std::string getCachePath()
{
    const char * ros_ws = std::getenv("ROS_WORKSPACE");  //NOLINT (concurrency-mt-unsafe)
    return (ros_ws != nullptr ? std::string(ros_ws) : "/workspaces/sailbot_workspace") +
           "/build/network_systems/projects/local_transceiver/global_waypoint_cache";
}

inline std::string getCacheTempPath()
{
    const char * ros_ws = std::getenv("ROS_WORKSPACE");  //NOLINT (concurrency-mt-unsafe)
    return (ros_ws != nullptr ? std::string(ros_ws) : "/workspaces/sailbot_workspace") +
           "/build/network_systems/projects/local_transceiver/global_waypoint_cache_temp";
}

static const std::string CACHE_PATH      = getCachePath();
static const std::string CACHE_TEMP_PATH = getCacheTempPath();

constexpr int NUM_BATTERIES = []() constexpr
{
    using batteries_arr = custom_interfaces::msg::Batteries::_batteries_type;
    return sizeof(batteries_arr) / sizeof(custom_interfaces::msg::HelperBattery);
}
();
constexpr int NUM_WIND_SENSORS = []() constexpr
{
    using wind_sensors_arr = custom_interfaces::msg::WindSensors::_wind_sensors_type;
    return sizeof(wind_sensors_arr) / sizeof(custom_interfaces::msg::WindSensor);
}
();

/****** Upper and lower bounds ******/

/***** Bounds for Latitude and Longitude ******/
constexpr float LAT_LBND = -90.0;
constexpr float LAT_UBND = 90.0;
constexpr float LON_LBND = -180.0;
constexpr float LON_UBND = 180.0;

/***** Bounds for Speed ******/
constexpr float SPEED_LBND = -10.0;  // Placeholder number
constexpr float SPEED_UBND = 10.0;   // Placeholder number

/***** Bounds for Heading ******/
constexpr float HEADING_LBND = 0.0;
constexpr float HEADING_UBND = 360.0;

// boat rotation
// See https://documentation.spire.com/ais-fundamentals/rate-of-turn-rot/ for how ROT works
constexpr int8_t ROT_LBND = -126;
constexpr int8_t ROT_UBND = 126;

// boat dimension
constexpr float SHIP_DIMENSION_LBND = 1;      // arbitrary number
constexpr float SHIP_DIMENSION_UBND = 650.0;  // arbitrary number

/***** Bounds for Battery ******/
constexpr float BATT_VOLT_LBND = 0.5;     // Placeholder number
constexpr float BATT_VOLT_UBND = 250.0;   // Placeholder number
constexpr float BATT_CURR_LBND = -200.0;  // Placeholder number
constexpr float BATT_CURR_UBND = 200.0;   // Placeholder number

/***** Bounds for Wind Sensor ******/
constexpr int WIND_DIRECTION_LBND = -180;
constexpr int WIND_DIRECTION_UBND = 179;
