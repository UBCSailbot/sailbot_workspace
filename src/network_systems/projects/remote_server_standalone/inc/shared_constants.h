// Copied from original, ROS dependencies removed
#pragma once
#include <string>
namespace SYSTEM_MODE
{
static const std::string PROD     = "production";
static const std::string DEV      = "development";
static const std::string TEST_SAT = "test_satellite";
};  // namespace SYSTEM_MODE
constexpr unsigned int MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES = 340;
constexpr unsigned int MAX_REMOTE_TO_LOCAL_PAYLOAD_SIZE_BYTES = 270;
constexpr float        LAT_LBND                               = -90.0;
constexpr float        LAT_UBND                               = 90.0;
constexpr float        LON_LBND                               = -180.0;
constexpr float        LON_UBND                               = 180.0;
constexpr float        BOAT_SPEED_LBND                        = 0.0;
constexpr float        BOAT_SPEED_UBND                        = 80.0;
constexpr float        WIND_SPEED_LBND                        = 0.0;
constexpr float        WIND_SPEED_UBND                        = 250.0;
constexpr float        SOG_SPEED_LBND                         = 0.0;
constexpr float        SOG_SPEED_UBND                         = 200.0;
constexpr float        HEADING_LBND                           = 0.0;
constexpr float        HEADING_UBND                           = 360.0;
constexpr float        TRIM_LBND                              = -40.0;
constexpr float        TRIM_UBND                              = 40.0;
constexpr int8_t       ROT_LBND                               = -126;
constexpr int8_t       ROT_UBND                               = 126;
constexpr float        SHIP_DIMENSION_LBND                    = 1;
constexpr float        SHIP_DIMENSION_UBND                    = 650.0;
constexpr float        BATT_VOLT_LBND                         = 0.5;
constexpr float        BATT_VOLT_UBND                         = 250.0;
constexpr float        BATT_CURR_LBND                         = -200.0;
constexpr float        BATT_CURR_UBND                         = 200.0;
constexpr int          WIND_DIRECTION_LBND                    = 0;
constexpr int          WIND_DIRECTION_UBND                    = 359;
