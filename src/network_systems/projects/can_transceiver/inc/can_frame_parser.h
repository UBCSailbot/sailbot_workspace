#pragma once

#include <linux/can.h>
#include <stdint.h>

#include <array>
#include <custom_interfaces/msg/batteries.hpp>
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
 *
 */
enum class CanId : canid_t {
    RESERVED               = 0x00,
    BMS_P_DATA_FRAME_1     = 0x31,
    BMS_P_DATA_FRAME_2     = 0x32,
    SAIL_WSM_CMD_FRAME_1   = 0x60,
    SAIL_WSM_CMD_FRAME_2   = 0x61,
    SAIL_ENCD_DATA_FRAME   = 0x62,
    SAIL_WSM_DATA_FRAME_1  = 0x63,
    SAIL_WSM_DATA_FRAME_2  = 0x64,
    SAIL_WIND_DATA_FRAME_1 = 0x65,
    SAIL_NAV_CMD_FRAME     = 0x66,
    RUDR_CMD_FRAME         = 0x70,
    RUDR_DATA_FRAME_1      = 0x71,
    RUDR_DATA_FRAME_2      = 0x72,
    PATH_GPS_DATA_FRAME_1  = 0x80,
    PATH_GPS_DATA_FRAME_2  = 0x81,
    PATH_GPS_DATA_FRAME_3  = 0x82,
    PATH_GPS_DATA_FRAME_4  = 0x83,
    PATH_WIND_DATA_FRAME   = 0x84,
};

/**
 * @brief Map the CanId enum to a description
 *
 */
static const std::map<CanId, std::string> CAN_DESC{
  {CanId::RESERVED, "RESERVED"},
  {CanId::BMS_P_DATA_FRAME_1, "BMS_P_DATA_FRAME_1 (Battery 1 data)"},
  {CanId::BMS_P_DATA_FRAME_2, "BMS_P_DATA_FRAME_2 (Battery 2 data)"},
  {CanId::SAIL_WSM_CMD_FRAME_1, "SAIL_WSM_CMD_FRAME_1 (Main sail command)"},
  {CanId::SAIL_WSM_CMD_FRAME_2, "SAIL_WSM_CMD_FRAME_2 (Jib Sail command)"},
  {CanId::SAIL_ENCD_DATA_FRAME, "SAIL_ENCD_DATA_FRAME (Sail encoder data)"},
  {CanId::SAIL_WSM_DATA_FRAME_1, "SAIL_WSM_DATA_FRAME_1 (Main sail data)"},
  {CanId::SAIL_WSM_DATA_FRAME_2, "SAIL_WSM_DATA_FRAME_2 (Jib sail data)"},
  {CanId::SAIL_WIND_DATA_FRAME_1, "SAIL_WIND_DATA_FRAME_1 (Mast wind sensor)"},
  {CanId::SAIL_NAV_CMD_FRAME, "SAIL_NAV_CMD_FRAME (Nav light commands)"},
  {CanId::RUDR_CMD_FRAME, "RUDR_CMD_FRAME (Rudder commands [BOTH RUDDERS])"},
  {CanId::RUDR_DATA_FRAME_1, "RUDR_DATA_FRAME_1 (Port rudder data)"},
  {CanId::RUDR_DATA_FRAME_2, "RUDR_DATA_FRAME_2 (Starboard rudder data)"},
  {CanId::PATH_GPS_DATA_FRAME_1, "PATH_GPS_DATA_FRAME_1 (GPS latitude)"},
  {CanId::PATH_GPS_DATA_FRAME_2, "PATH_GPS_DATA_FRAME_2 (GPS longitude)"},
  {CanId::PATH_GPS_DATA_FRAME_3, "PATH_GPS_DATA_FRAME_3 (GPS other data)"},
  {CanId::PATH_GPS_DATA_FRAME_4, "PATH_GPS_DATA_FRAME_4 (GPS time reporting [ex. day of the year])"},
  {CanId::PATH_WIND_DATA_FRAME, "PATH_WIND_DATA_FRAME (Hull wind sensor)"}};

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
    // Valid CanIds that a Battery object can have
    static constexpr std::array<CanId, 2> BATTERY_IDS       = {CanId::BMS_P_DATA_FRAME_1, CanId::BMS_P_DATA_FRAME_2};
    static constexpr uint8_t              CAN_BYTE_DLEN_    = 8;
    static constexpr uint8_t              BYTE_OFF_VOLT     = 0;
    static constexpr uint8_t              BYTE_OFF_CURR     = 2;
    static constexpr uint8_t              BYTE_OFF_MAX_VOLT = 4;
    static constexpr uint8_t              BYTE_OFF_MIN_VOLT = 6;

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
     * @param id      CanId of the battery (use the rosIdxToCanId() method if unknown)
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
     * @return A string that can be printed or logged to debug a Battery object
     */
    std::string debugStr() const override;

    /**
     * @brief Factory method to convert the index of a battery in the custom_interfaces ROS representation
     *        into a CanId if valid.
     *
     * @param bat_idx idx of the battery in a custom_interfaces::msg::Batteries array
     * @return CanId if valid, std::nullopt if invalid
     */
    static std::optional<CanId> rosIdxToCanId(size_t bat_idx);

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
    float volt_;      // Average voltage of cells in the battery
    float curr_;      // Current - positive means charging and negative means discharging (powering the boat)
    float volt_max_;  // Maximum voltage of cells in the battery pack (unused)
    float volt_min_;  // Minimum voltage of cells in the battery pack (unused)
};

}  // namespace CAN_FP
