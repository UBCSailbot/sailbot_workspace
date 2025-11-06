#pragma once

#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/ph_sensors.hpp>
#include <custom_interfaces/msg/pressure_sensors.hpp>
#include <custom_interfaces/msg/salinity_sensors.hpp>
#include <custom_interfaces/msg/temp_sensors.hpp>
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
constexpr int NUM_TEMP_SENSORS = []() constexpr
{
    using temp_sensors_arr = custom_interfaces::msg::TempSensors::_temp_sensors_type;
    return sizeof(temp_sensors_arr) / sizeof(custom_interfaces::msg::TempSensor);
}
();
constexpr int NUM_PH_SENSORS = []() constexpr
{
    using ph_sensors_arr = custom_interfaces::msg::PhSensors::_ph_sensors_type;
    return sizeof(ph_sensors_arr) / sizeof(custom_interfaces::msg::PhSensor);
}
();
constexpr int NUM_PRESSURE_SENSORS = []() constexpr
{
    using pressure_sensors_arr = custom_interfaces::msg::PressureSensors::_pressure_sensors_type;
    return sizeof(pressure_sensors_arr) / sizeof(custom_interfaces::msg::PressureSensor);
}
();
constexpr int NUM_SALINITY_SENSORS = []() constexpr
{
    using salinity_sensors_arr = custom_interfaces::msg::SalinitySensors::_salinity_sensors_type;
    return sizeof(salinity_sensors_arr) / sizeof(custom_interfaces::msg::SalinitySensor);
}
();

/****** Upper and lower bounds ******/

/***** Bounds for Latitude and Longitude ******/
constexpr float LAT_LBND = -90.0;
constexpr float LAT_UBND = 90.0;
constexpr float LON_LBND = -180.0;
constexpr float LON_UBND = 180.0;

/***** Bounds for Speed ******/
constexpr float BOAT_SPEED_LBND = 0.0;    // Arbitrary number (kmph)
constexpr float BOAT_SPEED_UBND = 80.0;   // Arbitrary number (kmph)
constexpr float WIND_SPEED_LBND = 0.0;    // Arbitrary number (kmph)
constexpr float WIND_SPEED_UBND = 250.0;  // Arbitrary number (kmph)
constexpr float SOG_SPEED_LBND  = 0.0;    // Arbitrary number (kmph)
constexpr float SOG_SPEED_UBND  = 200.0;  // Arbitrary number (kmph)

/***** Bounds for Heading ******/
constexpr float HEADING_LBND = 0.0;
constexpr float HEADING_UBND = 360.0;

/***** Bounds for Trim Tab *****/
constexpr float TRIM_LBND = -40.0;
constexpr float TRIM_UBND = 40.0;

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
constexpr int WIND_DIRECTION_LBND = 0;
constexpr int WIND_DIRECTION_UBND = 359;

/***** Bounds for Temp Sensor ******/
constexpr float TEMP_LBND = 73.15;   // lbnd of sensor being used (Kelvin)
constexpr float TEMP_UBND = 473.15;  // ubnd of sensor being used (Kelvin)

/***** Bounds for Ph Sensor ******/
constexpr float PH_LBND = -1.6;  // lbnd of sensor being used
constexpr float PH_UBND = 15.6;  // ubnd of sensor being used

/***** Bounds for Salinity Sensor ******/
constexpr float SALINITY_LBND = 0;        // lbnd of sensor being used is 0.07
constexpr float SALINITY_UBND = 1000000;  // ubnd of sensor being used is 500000+

/***** Bounds for Pressure Sensor ******/
constexpr float PRESSURE_LBND = -14.5;  // lowest lbnd of pressure sensors under consideration is -14.5 psi
constexpr float PRESSURE_UBND = 32.6;   // max int16_t, since ubnd of sensors under consideration is way higher
