#pragma once

#include <linux/can.h>
#include <stdint.h>

#include <array>
#include <custom_interfaces/msg/ais_ships.hpp>
#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/desired_heading.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <custom_interfaces/msg/helper_ais_ship.hpp>
#include <custom_interfaces/msg/ph_sensor.hpp>
#include <custom_interfaces/msg/pressure_sensor.hpp>
#include <custom_interfaces/msg/sail_cmd.hpp>
#include <custom_interfaces/msg/salinity_sensor.hpp>
#include <custom_interfaces/msg/temp_sensor.hpp>
#include <custom_interfaces/msg/wind_sensor.hpp>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>

// CAN frame definitions from: https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1827176527/CAN+Frames
namespace CAN_FP
{

using CanFrame   = struct canfd_frame;
using RawDataBuf = std::array<uint8_t, CANFD_MAX_DLEN>;
namespace msg    = custom_interfaces::msg;

/**
 * @brief IDs of CAN frames relevant to the Software team
 * NOTE: IDs are placeholders for now.
 *
 */
enum class CanId : canid_t {
    PWR_MODE              = 0x00,
    MAIN_HEADING          = 0x01,
    MAIN_TR_TAB           = 0x02,
    RESERVED              = 0x29,
    BMS_DATA_FRAME        = 0x30,
    SAIL_WIND             = 0x40,
    DATA_WIND             = 0x41,
    RUDDER_DATA_FRAME     = 0x50,
    SAIL_AIS              = 0x60,
    PATH_GPS_DATA_FRAME   = 0x70,
    TEMP_SENSOR_START     = 0x100,
    TEMP_1                = 0x101,
    TEMP_2                = 0x102,
    TEMP_3                = 0x103,
    TEMP_4                = 0x104,
    TEMP_5                = 0x105,
    TEMP_6                = 0x106,
    TEMP_7                = 0x107,
    TEMP_8                = 0x108,
    TEMP_9                = 0x109,
    TEMP_10               = 0x10A,
    TEMP_11               = 0x10B,
    TEMP_12               = 0x10C,
    TEMP_13               = 0x10D,
    TEMP_14               = 0x10E,
    TEMP_SENSOR_END       = 0x10F,
    PH_SENSOR_START       = 0x110,
    PH_1                  = 0x111,
    PH_2                  = 0x112,
    PH_3                  = 0x113,
    PH_4                  = 0x114,
    PH_5                  = 0x115,
    PH_6                  = 0x116,
    PH_7                  = 0x117,
    PH_8                  = 0x118,
    PH_9                  = 0x119,
    PH_10                 = 0x11A,
    PH_11                 = 0x11B,
    PH_12                 = 0x11C,
    PH_13                 = 0x11D,
    PH_14                 = 0x11E,
    PH_SENSOR_END         = 0x11F,
    SALINITY_SENSOR_START = 0x120,
    SALINITY_1            = 0x121,
    SALINITY_2            = 0x122,
    SALINITY_3            = 0x123,
    SALINITY_4            = 0x124,
    SALINITY_5            = 0x125,
    SALINITY_6            = 0x126,
    SALINITY_7            = 0x127,
    SALINITY_8            = 0x128,
    SALINITY_9            = 0x129,
    SALINITY_10           = 0x12A,
    SALINITY_11           = 0x12B,
    SALINITY_12           = 0x12C,
    SALINITY_13           = 0x12D,
    SALINITY_14           = 0x12E,
    SALINITY_SENSOR_END   = 0x12F,
    PRESSURE_SENSOR_START = 0x130,
    PRESSURE_1            = 0x131,
    PRESSURE_2            = 0x132,
    PRESSURE_3            = 0x133,
    PRESSURE_4            = 0x134,
    PRESSURE_5            = 0x135,
    PRESSURE_6            = 0x136,
    PRESSURE_7            = 0x137,
    PRESSURE_8            = 0x138,
    PRESSURE_9            = 0x139,
    PRESSURE_10           = 0x13A,
    PRESSURE_11           = 0x13B,
    PRESSURE_12           = 0x13C,
    PRESSURE_13           = 0x13D,
    PRESSURE_14           = 0x13E,
    PRESSURE_SENSOR_END   = 0x13F,
    GENERIC_SENSOR_START  = 0x140,
    GENERIC_SENSOR_END    = 0x1FF,
    CAN_MODE              = 0x210
};

inline bool isValidCanId(canid_t id)
{
    return id == static_cast<canid_t>(CanId::PWR_MODE) || id == static_cast<canid_t>(CanId::MAIN_HEADING) ||
           id == static_cast<canid_t>(CanId::MAIN_TR_TAB) || id == static_cast<canid_t>(CanId::RESERVED) ||
           id == static_cast<canid_t>(CanId::BMS_DATA_FRAME) || id == static_cast<canid_t>(CanId::SAIL_WIND) ||
           id == static_cast<canid_t>(CanId::DATA_WIND) || id == static_cast<canid_t>(CanId::RUDDER_DATA_FRAME) ||
           id == static_cast<canid_t>(CanId::SAIL_AIS) || id == static_cast<canid_t>(CanId::PATH_GPS_DATA_FRAME) ||

           (id >= static_cast<canid_t>(CanId::TEMP_SENSOR_START) &&
            id <= static_cast<canid_t>(CanId::TEMP_SENSOR_END)) ||

           (id >= static_cast<canid_t>(CanId::PH_SENSOR_START) && id <= static_cast<canid_t>(CanId::PH_SENSOR_END)) ||

           (id >= static_cast<canid_t>(CanId::SALINITY_SENSOR_START) &&
            id <= static_cast<canid_t>(CanId::SALINITY_SENSOR_END)) ||

           (id >= static_cast<canid_t>(CanId::PRESSURE_SENSOR_START) &&
            id <= static_cast<canid_t>(CanId::PRESSURE_SENSOR_END)) ||

           (id >= static_cast<canid_t>(CanId::GENERIC_SENSOR_START) &&
            id <= static_cast<canid_t>(CanId::GENERIC_SENSOR_END)) ||

           id == static_cast<canid_t>(CanId::CAN_MODE);
}

/**
 * @brief Map the CanId enum to a description
 *
 */
static const std::map<CanId, std::string> CAN_DESC{
  {CanId::PWR_MODE, "PWR_MODE (Power Mode)"},
  {CanId::MAIN_HEADING, "MAIN_HEADING (Main heading for rudder)"},
  {CanId::MAIN_TR_TAB, "MAIN_TR_TAB (Trim tab for sail)"},
  {CanId::BMS_DATA_FRAME, "BMS_P_DATA_FRAME (Battery data)"},
  {CanId::RESERVED, "Reserved for mainframe (0x0 - 0x29)"},
  {CanId::SAIL_AIS, "SAIL_AIS (AIS ship data)"},
  {CanId::SAIL_WIND, "SAIL_WIND (Mast wind sensor)"},
  {CanId::RUDDER_DATA_FRAME, "RUDDER_DATA_FRAME (Rudder data from ecompass)"},
  {CanId::PATH_GPS_DATA_FRAME, "PATH_GPS_DATA_FRAME (GPS latitude)"},
  {CanId::DATA_WIND, "DATA_WIND (Hull wind sensor)"},
  {CanId::TEMP_SENSOR_START, "TEMP_SENSOR_START (Start of temperature sensor range)"},
  {CanId::TEMP_1, "TEMP_1 (Temperature sensor #1)"},
  {CanId::TEMP_2, "TEMP_2 (Temperature sensor #2)"},
  {CanId::TEMP_3, "TEMP_3 (Temperature sensor #3)"},
  {CanId::TEMP_4, "TEMP_4 (Temperature sensor #4)"},
  {CanId::TEMP_5, "TEMP_5 (Temperature sensor #5)"},
  {CanId::TEMP_6, "TEMP_6 (Temperature sensor #6)"},
  {CanId::TEMP_7, "TEMP_7 (Temperature sensor #7)"},
  {CanId::TEMP_8, "TEMP_8 (Temperature sensor #8)"},
  {CanId::TEMP_9, "TEMP_9 (Temperature sensor #9)"},
  {CanId::TEMP_10, "TEMP_10 (Temperature sensor #10)"},
  {CanId::TEMP_11, "TEMP_11 (Temperature sensor #11)"},
  {CanId::TEMP_12, "TEMP_12 (Temperature sensor #12)"},
  {CanId::TEMP_13, "TEMP_13 (Temperature sensor #13)"},
  {CanId::TEMP_14, "TEMP_14 (Temperature sensor #14)"},
  {CanId::TEMP_SENSOR_END, "TEMP_SENSOR_END (End of temperature sensor range)"},
  {CanId::PH_SENSOR_START, "PH_SENSOR_START (Start of pH sensor range)"},
  {CanId::PH_1, "PH_1 (pH sensor #1)"},
  {CanId::PH_2, "PH_2 (pH sensor #2)"},
  {CanId::PH_3, "PH_3 (pH sensor #3)"},
  {CanId::PH_4, "PH_4 (pH sensor #4)"},
  {CanId::PH_5, "PH_5 (pH sensor #5)"},
  {CanId::PH_6, "PH_6 (pH sensor #6)"},
  {CanId::PH_7, "PH_7 (pH sensor #7)"},
  {CanId::PH_8, "PH_8 (pH sensor #8)"},
  {CanId::PH_9, "PH_9 (pH sensor #9)"},
  {CanId::PH_10, "PH_10 (pH sensor #10)"},
  {CanId::PH_11, "PH_11 (pH sensor #11)"},
  {CanId::PH_12, "PH_12 (pH sensor #12)"},
  {CanId::PH_13, "PH_13 (pH sensor #13)"},
  {CanId::PH_14, "PH_14 (pH sensor #14)"},
  {CanId::PH_SENSOR_END, "PH_SENSOR_END (End of pH sensor range)"},
  {CanId::SALINITY_SENSOR_START, "SALINITY_SENSOR_START (Start of salinity sensor range)"},
  {CanId::SALINITY_1, "SALINITY_1 (Salinity sensor #1)"},
  {CanId::SALINITY_2, "SALINITY_2 (Salinity sensor #2)"},
  {CanId::SALINITY_3, "SALINITY_3 (Salinity sensor #3)"},
  {CanId::SALINITY_4, "SALINITY_4 (Salinity sensor #4)"},
  {CanId::SALINITY_5, "SALINITY_5 (Salinity sensor #5)"},
  {CanId::SALINITY_6, "SALINITY_6 (Salinity sensor #6)"},
  {CanId::SALINITY_7, "SALINITY_7 (Salinity sensor #7)"},
  {CanId::SALINITY_8, "SALINITY_8 (Salinity sensor #8)"},
  {CanId::SALINITY_9, "SALINITY_9 (Salinity sensor #9)"},
  {CanId::SALINITY_10, "SALINITY_10 (Salinity sensor #10)"},
  {CanId::SALINITY_11, "SALINITY_11 (Salinity sensor #11)"},
  {CanId::SALINITY_12, "SALINITY_12 (Salinity sensor #12)"},
  {CanId::SALINITY_13, "SALINITY_13 (Salinity sensor #13)"},
  {CanId::SALINITY_14, "SALINITY_14 (Salinity sensor #14)"},
  {CanId::SALINITY_SENSOR_END, "SALINITY_SENSOR_END (End of salinity sensor range)"},
  {CanId::PRESSURE_SENSOR_START, "PRESSURE_SENSOR_START (Start of pressure sensor range)"},
  {CanId::PRESSURE_1, "PRESSURE_1 (Pressure sensor #1)"},
  {CanId::PRESSURE_2, "PRESSURE_2 (Pressure sensor #2)"},
  {CanId::PRESSURE_3, "PRESSURE_3 (Pressure sensor #3)"},
  {CanId::PRESSURE_4, "PRESSURE_4 (Pressure sensor #4)"},
  {CanId::PRESSURE_5, "PRESSURE_5 (Pressure sensor #5)"},
  {CanId::PRESSURE_6, "PRESSURE_6 (Pressure sensor #6)"},
  {CanId::PRESSURE_7, "PRESSURE_7 (Pressure sensor #7)"},
  {CanId::PRESSURE_8, "PRESSURE_8 (Pressure sensor #8)"},
  {CanId::PRESSURE_9, "PRESSURE_9 (Pressure sensor #9)"},
  {CanId::PRESSURE_10, "PRESSURE_10 (Pressure sensor #10)"},
  {CanId::PRESSURE_11, "PRESSURE_11 (Pressure sensor #11)"},
  {CanId::PRESSURE_12, "PRESSURE_12 (Pressure sensor #12)"},
  {CanId::PRESSURE_13, "PRESSURE_13 (Pressure sensor #13)"},
  {CanId::PRESSURE_14, "PRESSURE_14 (Pressure sensor #14)"},
  {CanId::PRESSURE_SENSOR_END, "PRESSURE_SENSOR_END (End of pressure sensor range)"},
  {CanId::CAN_MODE, "CAN_MODE (When mode is MANUAL no messages sent on the CAN bus)"}};

/**
 * @brief Custom exception for when an attempt is made to construct a CAN object with a mismatched ID
 *
 */
class CanIdMismatchException : public std::exception
{
public:
    /**
     * @brief Instantiate the CanIdMismatchException
     *
     * @param valid_ids Expected IDs
     * @param received  The invalid ID that was received
     */
    CanIdMismatchException(std::span<const CanId> valid_ids, CanId received);

    using std::exception::what;  // Needed to resolve virtual function overload error
    /**
     * @brief Return the exception message
     *
     * @return exception message
     */
    const char * what();

private:
    std::string msg_;  // exception message
};

/**
 * @brief Abstract class that represents a generic dataframe. Not meant to be instantiated
 * on its own, and must be instantiated with a derived class
 *
 */
class BaseFrame
{
public:
    const CanId   id_;
    const uint8_t can_byte_dlen_;  // Number of bytes of data used in the Linux CAN representation

    /**
     * @brief Override the << operator for printing
     *
     * @param os  output stream (typically std::cout)
     * @param can BaseFrame instance to print
     * @return stream to print
     */
    friend std::ostream & operator<<(std::ostream & os, const BaseFrame & can);

protected:
    /**
     * @brief Derived classes can instantiate a base frame using an CanId and a data length
     *
     * @param id            CanId of the fraeme
     * @param can_byte_dlen Number of bytes used in the Linux CAN representation
     */
    BaseFrame(CanId id, uint8_t can_byte_dlen);

    /**
     * @brief Derived classes can instantiate a base frame and check if the id is vaild
     *
     * @param valid_ids      a span of valid CanIds
     * @param id             the id to check
     * @param can_byte_dlen_ Number of bytes used in the Linux CAN representatio nof the dataframe
     */
    BaseFrame(std::span<const CanId> valid_ids, CanId id, uint8_t can_byte_dlen_);

    /**
     * @return The Linux CanFrame representation of the frame
     */
    virtual CanFrame toLinuxCan() const;

    /**
     * @return A string that can be printed or logged for debugging
     */
    virtual std::string debugStr() const;

    /**
     * @brief A string representation of the dataframe
     *
     */
    virtual std::string toString() const;
};

/**
 * @brief A battery class derived from the BaseFrame. Represents battery data.
 *
 */
class Battery final : public BaseFrame
{
public:
    // Valid CanIds that a Battery object can have. There used to be two, but now there is only one.
    static constexpr std::array<CanId, 1> BATTERY_IDS    = {CanId::BMS_DATA_FRAME};
    static constexpr uint8_t              CAN_BYTE_DLEN_ = 8;
    static constexpr uint8_t              BYTE_OFF_VOLT  = 0;
    static constexpr uint8_t              BYTE_OFF_CURR  = 4;

    /**
     * @brief Explicitly deleted no-argument constructor
     *
     */
    Battery() = delete;

    /**
     * @brief Construct a Battery object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit Battery(const CanFrame & cf);

    /**
     * @brief Construct a Battery object from a custom_interfaces ROS msg representation
     *
     * @param ros_bat custom_interfaces representation of a Battery
     * @param id      CanId of the battery
     */
    explicit Battery(msg::HelperBattery ros_bat, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the Battery object
     */
    msg::HelperBattery toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the Battery object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug an Battery object
     */
    std::string debugStr() const override;

    /**
     * @brief A string representation of the Battery object
     *
     */
    std::string toString() const override;

private:
    /**
     * @brief Private helper constructor for Battery objects
     *
     * @param id CanId of the battery
     */
    explicit Battery(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a Battery object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;

    // Note: Each BMS battery is comprised of multiple battery cells
    float volt_;  // Average voltage of cells in the battery
    float curr_;  // Current - positive means charging and negative means discharging (powering the boat)
};

/**
 * @brief A sail class derived from the BaseFrame. Represents a trim tab command.
 *
 */
class MainTrimTab final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 1> TRIM_TAB_IDS   = {CanId::MAIN_TR_TAB};
    static constexpr uint8_t              CAN_BYTE_DLEN_ = 4;
    static constexpr uint8_t              BYTE_OFF_ANGLE = 0;

    /**
     * @brief Explicitly deleted no-argument constructor
     *
     */
    MainTrimTab() = delete;

    /**
     * @brief Construct a MainTrimTab object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit MainTrimTab(const CanFrame & cf);

    /**
     * @brief Construct a MainTrimTab object from a custom_interfaces ROS msg representation
     *
     * @param ros_sail_cmd custom_interfaces representation of a MainTrimTab
     * @param id      CanId of the MainTrimTab
     */
    explicit MainTrimTab(msg::SailCmd ros_sail_cmd, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the MainTrimTab object
     */
    msg::SailCmd toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the MainTrimTab object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a MainTrimTab object
     */
    std::string debugStr() const override;

    /**
     * @brief A string representation of the MainTrimTab object
     *
     */
    std::string toString() const override;

private:
    /**
     * @brief Private helper constructor for MainTrimTab objects
     *
     * @param id CanId of the MainTrimTab
     */
    explicit MainTrimTab(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a MainTrimTab object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;

    float angle_;  // Angle specified by the command
};

/**
 * @brief A wind class derived from the BaseFrame. Represents wind data.
 *
 */
class WindSensor final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 2> WIND_SENSOR_IDS = {CanId::SAIL_WIND, CanId::DATA_WIND};
    static constexpr uint8_t              CAN_BYTE_DLEN_  = 4;
    static constexpr uint8_t              BYTE_OFF_ANGLE  = 0;
    static constexpr uint8_t              BYTE_OFF_SPEED  = 2;

    /**
     * @brief Explicitly deleted no-argument constructor
     *
     */
    WindSensor() = delete;

    /**
     * @brief Construct a Wind object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit WindSensor(const CanFrame & cf);

    /**
     * @brief Construct a WindSensor object from a custom_interfaces ROS msg representation
     *
     * @param ros_wind_sensor custom_interfaces representation of a WindSensor
     * @param id      CanId of the WindSensor (use the rosIdxToCanId() method if unknown)
     */
    explicit WindSensor(msg::WindSensor ros_wind_sensor, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the WindSensor
     */
    msg::WindSensor toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the WindSensor object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a WindSensor object
     */
    std::string debugStr() const override;

    /**
     * @brief A string representation of the WindSensor object
     *
     */
    std::string toString() const override;

    /**
     * @brief Factory method to convert the index of a wind sensor in the custom_interfaces ROS representation
     *        into a CanId if valid.
     *
     * @param wind_idx idx of the wind sensor in a custom_interfaces::msg::WindSensors array
     * @return CanId if valid, std::nullopt if invalid
     */
    static std::optional<CanId> rosIdxToCanId(size_t wind_idx);

private:
    /**
     * @brief Private helper constructor for WindSensor objects
     *
     * @param id CanId of the WindSensor Object
     */
    explicit WindSensor(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a WindSensor object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;

    int16_t wind_angle_;
    float   wind_speed_;
};

/**
 * @brief GPS class derived from the BaseFrame. Represents GPS data.
 *
 */
class GPS final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 1> GPS_IDS        = {CanId::PATH_GPS_DATA_FRAME};
    static constexpr uint8_t              CAN_BYTE_DLEN_ = 20;
    static constexpr uint32_t             BYTE_OFF_LAT   = 0;
    static constexpr uint32_t             BYTE_OFF_LON   = 4;
    static constexpr uint32_t             BYTE_OFF_SEC   = 8;
    static constexpr uint32_t             BYTE_OFF_MIN   = 12;
    static constexpr uint32_t             BYTE_OFF_HOUR  = 13;
    static constexpr uint32_t             BYTE_OFF_RESV  = 14;
    static constexpr uint32_t             BYTE_OFF_SPEED = 16;

    /**
       * @brief Explicitly deleted no-argument constructor
       *
       */
    GPS() = delete;

    /**
     * @brief Construct a GPS object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit GPS(const CanFrame & cf);

    /**
     * @brief Construct a GPS object from a custom_interfaces ROS msg representation
     *
     * @param ros_gps custom_interfaces representation of a GPS
     * @param id      CanId of the GPS
     */
    explicit GPS(msg::GPS ros_gps, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the GPS object
     */
    msg::GPS toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the GPS object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a GPS object
     */
    std::string debugStr() const override;

    /**
     * @brief A string representation of the GPS object
     *
     */
    std::string toString() const override;

private:
    /**
     * @brief Private helper constructor for GPS objects
     *
     * @param id CanId of the GPS
     */
    explicit GPS(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a GPS object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;

    float lat_;
    float lon_;
    float sec_;
    float min_;
    float hour_;
    float speed_;
};

/**
 * @brief AISShips class derived from BaseFrame. Represents AIS ship data.
 *
 */
class AISShips final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 1> AISSHIPS_IDS       = {CanId::SAIL_AIS};
    static constexpr uint32_t             CAN_BYTE_DLEN_     = 26;
    static constexpr uint32_t             BYTE_OFF_ID        = 0;
    static constexpr uint32_t             BYTE_OFF_LAT       = 4;
    static constexpr uint32_t             BYTE_OFF_LON       = 8;
    static constexpr uint16_t             BYTE_OFF_SPEED     = 12;
    static constexpr uint16_t             BYTE_OFF_COURSE    = 14;
    static constexpr uint16_t             BYTE_OFF_HEADING   = 16;
    static constexpr uint8_t              BYTE_OFF_ROT       = 18;
    static constexpr uint16_t             BYTE_OFF_LENGTH    = 19;
    static constexpr uint8_t              BYTE_OFF_WIDTH     = 21;
    static constexpr uint8_t              BYTE_OFF_IDX       = 23;
    static constexpr uint8_t              BYTE_OFF_NUM_SHIPS = 24;

    /**
     * @brief Explicitly deleted no-argument constructor
     *
     */
    AISShips() = delete;

    /**
     * @brief Construct an AISShips object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit AISShips(const CanFrame & cf);

    /**
     * @brief Construct an AISShips object from a custom_interfaces ROS msg representation
     *
     * @param ros_ais_ship custom_interfaces representation of an AISShip
     * @param id      CanId of the AISShips
     */
    explicit AISShips(msg::HelperAISShip ros_ship, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the AISShips object
     */
    msg::HelperAISShip toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the Battery object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a Battery object
     */
    std::string debugStr() const override;

    /**
     * @brief A string representation of the AISShips object
     *
     */
    std::string toString() const override;

    /**
     * @brief Returns the number of ships
     *
     * @return the number of ships
     */
    int getNumShips() const { return num_ships_; }

    /**
     * @brief Returns the index of the current ship
     *
     * @return the index of the current ship
     */
    int getShipIndex() const { return idx_; }

private:
    /**
     * @brief Private helper constructor for AISShips objects
     *
     * @param id CanId of the AISShips
     */
    explicit AISShips(CanId id);

    /**
     * @brief Rounds a float to 4 decimal places of precision
     *
     * @param val
     * @return rounded float
     */
    static float roundFloat(float val);

    /**
     * @brief Check if the assigned fields after constructing an AISShips object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;

    uint8_t  num_ships_;
    float    lat_;
    float    lon_;
    float    speed_;
    int8_t   rot_;
    float    course_;
    float    heading_;
    float    width_;
    float    length_;
    uint32_t ship_id_;
    uint8_t  idx_;
};

/**
 * @brief Power mode class derived from BaseFrame. Represents power mode data.
 *
 */
class PwrMode final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 1>   PWR_MODE_IDS      = {CanId::PWR_MODE};
    static constexpr uint8_t                CAN_BYTE_DLEN_    = 1;
    static constexpr uint8_t                BYTE_OFF_MODE     = 0;
    static constexpr uint8_t                POWER_MODE_LOW    = 0;
    static constexpr uint8_t                POWER_MODE_NORMAL = 1;
    static constexpr std::array<uint8_t, 2> PWR_MODES         = {POWER_MODE_LOW, POWER_MODE_NORMAL};

    /**
     * @brief Explicitly deleted no-argument constructor
     *
     */
    PwrMode() = delete;

    /**
     * @brief Construct an PwrMode object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit PwrMode(const CanFrame & cf);

    /**
     * @brief Construct a PwrMode object given a mode and CAN ID
     *
     * @param mode    Power mode select
     * @param id      CanId of the PwrMode
     */
    explicit PwrMode(uint8_t mode, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the PwrMode object
     */
    //msg::HelperPwrMode toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the PwrMode object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a PwrMode object
     */
    std::string debugStr() const override;

private:
    /**
     * @brief Private helper constructor for PwrMode objects
     *
     * @param id CanId of the PwrMode
     */
    explicit PwrMode(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a PwrMode object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;

    uint8_t mode_;
};

/**
 * @brief A DesiredHeading class derived from the BaseFrame. Represents a desired heading for the rudder.
 *
 */
class DesiredHeading final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 1> DESIRED_HEADING_IDS = {CanId::MAIN_HEADING};
    static constexpr uint8_t              CAN_BYTE_DLEN_      = 5;
    static constexpr uint8_t              BYTE_OFF_HEADING    = 0;
    static constexpr uint8_t              BYTE_OFF_STEERING   = 4;

    /**
     * @brief Explicitly deleted no-argument constructor
     *
     */
    DesiredHeading() = delete;

    /**
     * @brief Construct a Desiredheading object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit DesiredHeading(const CanFrame & cf);

    /**
     * @brief Construct a DesiredHeading object from a custom_interfaces ROS msg representation
     *
     * @param ros_desired_heading custom_interfaces representation of a DesiredHeading
     * @param id      CanId of the DesiredHeading
     */
    explicit DesiredHeading(msg::DesiredHeading ros_desired_heading, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the DesiredHeading object
     */
    msg::DesiredHeading toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the DesiredHeading object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a DesiredHeading object
     */
    std::string debugStr() const override;

    /**
     * @brief A string representation of the DesiredHeading object
     *
     */
    std::string toString() const override;

private:
    /**
     * @brief Private helper constructor for DesiredHeading objects
     *
     * @param id CanId of the DesiredHeading
     */
    explicit DesiredHeading(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a DesiredHeading object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;

    float   heading_;  // Angle specified by the command
    uint8_t steering_;
};

/**
 * @brief A Rudder Data class derived from the BaseFrame. Represents data of rudder's position.
 *
 */
class RudderData final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 1> RUDDER_DATA_IDS  = {CanId::RUDDER_DATA_FRAME};
    static constexpr uint8_t              CAN_BYTE_DLEN_   = 4;
    static constexpr uint8_t              BYTE_OFF_HEADING = 0;

    /**
     * @brief Explicitly deleted no-argument constructor
     *
     */
    RudderData() = delete;

    /**
     * @brief Construct a RudderData object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit RudderData(const CanFrame & cf);

    /**
     * @brief Construct a HelperHeading object from a custom_interfaces ROS msg representation
     *
     * @param ros_rudder_heading custom_interfaces representation of a HelperHeading
     * @param id      CanId of the HelperHeading
     */
    explicit RudderData(msg::HelperHeading ros_rudder_data, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the DesiredHeading object
     */
    msg::HelperHeading toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the RudderData object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a RudderData object
     */
    std::string debugStr() const override;

    /**
      * @brief A string representation of the RudderData object
      *
      */
    std::string toString() const override;

private:
    /**
     * @brief Private helper constructor for DesiredHeading objects
     *
     * @param id CanId of the RudderData
     */
    explicit RudderData(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a DesiredHeading object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;

    float heading_;
};

/**
 * @brief A Temp Sensor class derived from the BaseFrame. Represents temperature data.
 *
 */
class TempSensor final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 16> TEMP_SENSOR_IDS = {
      CanId::TEMP_SENSOR_START,
      CanId::TEMP_1,
      CanId::TEMP_2,
      CanId::TEMP_3,
      CanId::TEMP_4,
      CanId::TEMP_5,
      CanId::TEMP_6,
      CanId::TEMP_7,
      CanId::TEMP_8,
      CanId::TEMP_9,
      CanId::TEMP_10,
      CanId::TEMP_11,
      CanId::TEMP_12,
      CanId::TEMP_13,
      CanId::TEMP_14,
      CanId::TEMP_SENSOR_END};
    static constexpr uint8_t CAN_BYTE_DLEN_ = 2;
    static constexpr uint8_t BYTE_OFF_TEMP  = 0;

    /**
      * @brief Explicitly deleted no-argument constructor
      *
      */
    TempSensor() = delete;

    /**
      * @brief Construct a Temp object from a Linux CanFrame representation
      *
      * @param cf Linux CanFrame
      */
    explicit TempSensor(const CanFrame & cf);

    /**
      * @brief Construct a TempSensor object from a custom_interfaces ROS msg representation
      *
      * @param ros_temp_sensor custom_interfaces representation of a TempSensor
      * @param id      CanId of the TempSensor (use the rosIdxToCanId() method if unknown)
      */
    explicit TempSensor(msg::TempSensor ros_temp_sensor, CanId id);

    /**
      * @return the custom_interfaces ROS representation of the TempSensor
      */
    msg::TempSensor toRosMsg() const;

    /**
      * @return the Linux CanFrame representation of the TempSensor object
      */
    CanFrame toLinuxCan() const override;

    /**
      * @return A string that can be printed or logged to debug a TempSensor object
      */
    std::string debugStr() const override;

    /**
      * @brief A string representation of the TempSensor object
      *
      */
    std::string toString() const override;

    /**
     * @brief Factory method to convert the index of a temp sensor in the custom_interfaces ROS representation
     *        into a CanId if valid.
     *
     * @param temp_idx idx of the temp sensor in a custom_interfaces::msg::TempSensors array
     * @return CanId if valid, std::nullopt if invalid
     */
    static std::optional<CanId> rosIdxToCanId(size_t temp_idx);

private:
    /**
      * @brief Private helper constructor for Temp objects
      *
      * @param id CanId of the TempSensor Object
      */
    explicit TempSensor(CanId id);

    /**
      * @brief Check if the assigned fields after constructing a temp object are within bounds.
      * @throws std::out_of_range if any assigned fields are outside of expected bounds
      */
    void checkBounds() const;

    float temp_;
};

/**
 * @brief A Ph Sensor class derived from the BaseFrame. Represents pH data.
 *
 */
class PhSensor final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 16> PH_SENSOR_IDS = {
      CanId::PH_SENSOR_START,
      CanId::PH_1,
      CanId::PH_2,
      CanId::PH_3,
      CanId::PH_4,
      CanId::PH_5,
      CanId::PH_6,
      CanId::PH_7,
      CanId::PH_8,
      CanId::PH_9,
      CanId::PH_10,
      CanId::PH_11,
      CanId::PH_12,
      CanId::PH_13,
      CanId::PH_14,
      CanId::PH_SENSOR_END};
    static constexpr uint8_t CAN_BYTE_DLEN_ = 2;
    static constexpr uint8_t BYTE_OFF_PH    = 0;

    /**
       * @brief Explicitly deleted no-argument constructor
       *
       */
    PhSensor() = delete;

    /**
       * @brief Construct a Ph object from a Linux CanFrame representation
       *
       * @param cf Linux CanFrame
       */
    explicit PhSensor(const CanFrame & cf);

    /**
       * @brief Construct a PhSensor object from a custom_interfaces ROS msg representation
       *
       * @param ros_ph_sensor custom_interfaces representation of a PhSensor
       * @param id      CanId of the GPS (use the rosIdxToCanId() method if unknown)
       */
    explicit PhSensor(msg::PhSensor ros_ph_sensor, CanId id);

    /**
       * @return the custom_interfaces ROS representation of the PhSensor
       */
    msg::PhSensor toRosMsg() const;

    /**
       * @return the Linux CanFrame representation of the PhSensor object
       */
    CanFrame toLinuxCan() const override;

    /**
       * @return A string that can be printed or logged to debug a PhSensor object
       */
    std::string debugStr() const override;

    /**
       * @brief A string representation of the PhSensor object
       *
       */
    std::string toString() const override;

    /**
     * @brief Factory method to convert the index of a ph sensor in the custom_interfaces ROS representation
     *        into a CanId if valid.
     *
     * @param ph_idx idx of the ph sensor in a custom_interfaces::msg::PhSensors array
     * @return CanId if valid, std::nullopt if invalid
     */
    static std::optional<CanId> rosIdxToCanId(size_t ph_idx);

private:
    /**
       * @brief Private helper constructor for Ph objects
       *
       * @param id CanId of the PhSensor Object
       */
    explicit PhSensor(CanId id);

    /**
       * @brief Check if the assigned fields after constructing a Ph object are within bounds.
       * @throws std::out_of_range if any assigned fields are outside of expected bounds
       */
    void checkBounds() const;

    float ph_;
};

/**
 * @brief A Salinity Sensor class derived from the BaseFrame. Represents salinity data.
 *
 */
class SalinitySensor final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 16> SALINITY_SENSOR_IDS = {
      CanId::SALINITY_SENSOR_START,
      CanId::SALINITY_1,
      CanId::SALINITY_2,
      CanId::SALINITY_3,
      CanId::SALINITY_4,
      CanId::SALINITY_5,
      CanId::SALINITY_6,
      CanId::SALINITY_7,
      CanId::SALINITY_8,
      CanId::SALINITY_9,
      CanId::SALINITY_10,
      CanId::SALINITY_11,
      CanId::SALINITY_12,
      CanId::SALINITY_13,
      CanId::SALINITY_14,
      CanId::SALINITY_SENSOR_END};
    static constexpr uint8_t CAN_BYTE_DLEN_    = 4;
    static constexpr uint8_t BYTE_OFF_SALINITY = 0;

    /**
        * @brief Explicitly deleted no-argument constructor
        *
        */
    SalinitySensor() = delete;

    /**
        * @brief Construct a Salinity object from a Linux CanFrame representation
        *
        * @param cf Linux CanFrame
        */
    explicit SalinitySensor(const CanFrame & cf);

    /**
        * @brief Construct a SalinitySensor object from a custom_interfaces ROS msg representation
        *
        * @param ros_salinity_sensor custom_interfaces representation of a SalinitySensor
        * @param id      CanId of the GPS (use the rosIdxToCanId() method if unknown)
        */
    explicit SalinitySensor(msg::SalinitySensor ros_salinity_sensor, CanId id);

    /**
        * @return the custom_interfaces ROS representation of the SalinitySensor
        */
    msg::SalinitySensor toRosMsg() const;

    /**
        * @return the Linux CanFrame representation of the SalinitySensor object
        */
    CanFrame toLinuxCan() const override;

    /**
        * @return A string that can be printed or logged to debug a SalinitySensor object
        */
    std::string debugStr() const override;

    /**
        * @brief A string representation of the SalinitySensor object
        *
        */
    std::string toString() const override;

    /**
     * @brief Factory method to convert the index of a salinity sensor in the custom_interfaces ROS representation
     *        into a CanId if valid.
     *
     * @param salinity_idx idx of the salinity sensor in a custom_interfaces::msg::SalinitySensors array
     * @return CanId if valid, std::nullopt if invalid
     */
    static std::optional<CanId> rosIdxToCanId(size_t salinity_idx);

private:
    /**
        * @brief Private helper constructor for Salinity objects
        *
        * @param id CanId of the SalinitySensor Object
        */
    explicit SalinitySensor(CanId id);

    /**
        * @brief Check if the assigned fields after constructing a Salinity object are within bounds.
        * @throws std::out_of_range if any assigned fields are outside of expected bounds
        */
    void checkBounds() const;

    float salinity_;
};

class PressureSensor final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 16> PRESSURE_SENSOR_IDS = {
      CanId::PRESSURE_SENSOR_START,
      CanId::PRESSURE_1,
      CanId::PRESSURE_2,
      CanId::PRESSURE_3,
      CanId::PRESSURE_4,
      CanId::PRESSURE_5,
      CanId::PRESSURE_6,
      CanId::PRESSURE_7,
      CanId::PRESSURE_8,
      CanId::PRESSURE_9,
      CanId::PRESSURE_10,
      CanId::PRESSURE_11,
      CanId::PRESSURE_12,
      CanId::PRESSURE_13,
      CanId::PRESSURE_14,
      CanId::PRESSURE_SENSOR_END};
    static constexpr uint8_t CAN_BYTE_DLEN_    = 4;
    static constexpr uint8_t BYTE_OFF_PRESSURE = 0;

    /**
        * @brief Explicitly deleted no-argument constructor
        *
        */
    PressureSensor() = delete;

    /**
        * @brief Construct a Pressure object from a Linux CanFrame representation
        *
        * @param cf Linux CanFrame
        */
    explicit PressureSensor(const CanFrame & cf);

    /**
        * @brief Construct a PressureSensor object from a custom_interfaces ROS msg representation
        *
        * @param ros_pressure_sensor custom_interfaces representation of a PressureSensor
        * @param id      CanId of the GPS (use the rosIdxToCanId() method if unknown)
        */
    explicit PressureSensor(msg::PressureSensor ros_pressure_sensor, CanId id);

    /**
        * @return the custom_interfaces ROS representation of the PressureSensor
        */
    msg::PressureSensor toRosMsg() const;

    /**
        * @return the Linux CanFrame representation of the Pressure object
        */
    CanFrame toLinuxCan() const override;

    /**
        * @return A string that can be printed or logged to debug a Pressure object
        */
    std::string debugStr() const override;

    /**
        * @brief A string representation of the PressureSensor object
        *
        */
    std::string toString() const override;

    /**
     * @brief Factory method to convert the index of a pressure sensor in the custom_interfaces ROS representation
     *        into a CanId if valid.
     *
     * @param pressure_idx idx of the pressure sensor in a custom_interfaces::msg::PressureSensors array
     * @return CanId if valid, std::nullopt if invalid
     */
    static std::optional<CanId> rosIdxToCanId(size_t pressure_idx);

private:
    /**
        * @brief Private helper constructor for Pressure objects
        *
        * @param id CanId of the PressureSensor Object
        */
    explicit PressureSensor(CanId id);

    /**
        * @brief Check if the assigned fields after constructing a Pressure object are within bounds.
        * @throws std::out_of_range if any assigned fields are outside of expected bounds
        */
    void checkBounds() const;

    float pressure_;
};

class CanMode final : public BaseFrame
{
public:
    static constexpr std::array<CanId, 1>   CAN_MODE_IDS    = {CanId::CAN_MODE};
    static constexpr uint8_t                CAN_BYTE_DLEN_  = 1;
    static constexpr uint8_t                BYTE_OFF_MODE   = 0;
    static constexpr uint8_t                CAN_MODE_NORMAL = 0;
    static constexpr uint8_t                CAN_MODE_MANUAL = 1;
    static constexpr std::array<uint8_t, 2> CAN_MODES       = {CAN_MODE_NORMAL, CAN_MODE_MANUAL};

    /**
     * @brief Explicitly deleted no-argument constructor
     *
     */
    CanMode() = delete;

    /**
     * @brief Construct an CanMode object from a Linux CanFrame representation
     *
     * @param cf Linux CanFrame
     */
    explicit CanMode(const CanFrame & cf);

    /**
     * @brief Construct a CanMode object given a mode and CAN ID
     *
     * @param mode    Power mode select
     * @param id      CanId of the CanMode
     */
    explicit CanMode(uint8_t mode, CanId id);

    /**
     * @return the Linux CanFrame representation of the CanMode object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a CanMode object
     */
    std::string debugStr() const override;

    /**
     * @brief A string representation of the CanMode object
     *
     */
    std::string toString() const override;

    uint8_t mode_;

private:
    /**
     * @brief Private helper constructor for CanMode objects
     *
     * @param id CanId of the CanMode
     */
    explicit CanMode(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a CanMode object are within bounds.
     * @throws std::out_of_range if any assigned fields are outside of expected bounds
     */
    void checkBounds() const;
};

}  // namespace CAN_FP
