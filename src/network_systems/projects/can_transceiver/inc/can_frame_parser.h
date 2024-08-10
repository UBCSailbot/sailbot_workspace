#pragma once

#include <linux/can.h>
#include <stdint.h>

#include <array>
#include <custom_interfaces/msg/ais_ships.hpp>
#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/desired_heading.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <custom_interfaces/msg/helper_ais_ship.hpp>
#include <custom_interfaces/msg/sail_cmd.hpp>
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
    PWR_MODE             = 0x00,
    MAIN_HEADING         = 0x01,
    MAIN_TR_TAB          = 0x02,
    RESERVED             = 0x29,
    BMS_DATA_FRAME       = 0x30,
    SAIL_WIND            = 0x40,
    DATA_WIND            = 0x41,
    RUDDER_DATA_FRAME    = 0x50,
    SAIL_AIS             = 0x60,
    PATH_GPS_DATA_FRAME  = 0x70,
    GENERIC_SENSOR_START = 0x100,
    GENERIC_SENSOR_END   = 0x1FF
};

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
  {CanId::MAIN_HEADING, "MAIN_HEADING (Main rudder command)"},
  {CanId::SAIL_WIND, "SAIL_WIND (Mast wind sensor)"},
  {CanId::RUDDER_DATA_FRAME, "RUDDER_DATA_FRAME (Rudder data from ecompass)"},
  {CanId::PATH_GPS_DATA_FRAME, "PATH_GPS_DATA_FRAME (GPS latitude)"},
  {CanId::DATA_WIND, "DATA_WIND (Hull wind sensor)"}};

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
     * @param id      CanId of the GPS (use the rosIdxToCanId() method if unknown)
     */
    explicit WindSensor(msg::WindSensor ros_wind_sensor, CanId id);

    /**
     * @return the custom_interfaces ROS representation of the WindSensor
     */
    msg::WindSensor toRosMsg() const;

    /**
     * @return the Linux CanFrame representation of the GPS object
     */
    CanFrame toLinuxCan() const override;

    /**
     * @return A string that can be printed or logged to debug a GPS object
     */
    std::string debugStr() const override;

    /**
     * @brief Factory method to convert the index of a wind sensor in the custom_interfaces ROS representation
     *        into a CanId if valid.
     *
     * @param bat_idx idx of the wind sensor in a custom_interfaces::msg::WindSensors array
     * @return CanId if valid, std::nullopt if invalid
     */
    static std::optional<CanId> rosIdxToCanId(size_t wind_idx);

private:
    /**
     * @brief Private helper constructor for GPS objects
     *
     * @param id CanId of the WindSensor Object
     */
    explicit WindSensor(CanId id);

    /**
     * @brief Check if the assigned fields after constructing a GPS object are within bounds.
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
    static constexpr uint16_t             BYTE_OFF_LENGTH    = 20;
    static constexpr uint8_t              BYTE_OFF_WIDTH     = 22;
    static constexpr uint8_t              BYTE_OFF_IDX       = 24;
    static constexpr uint8_t              BYTE_OFF_NUM_SHIPS = 25;

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
    static constexpr uint8_t              CAN_BYTE_DLEN_      = 4;
    static constexpr uint8_t              BYTE_OFF_HEADING    = 0;

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

    float heading_;  // Angle specified by the command
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

}  // namespace CAN_FP
