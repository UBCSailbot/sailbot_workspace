#pragma once

#include <boost/asio/streambuf.hpp>
#include <string>

#include "at_cmds.h"
#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#include "custom_interfaces/msg/ais_ships.hpp"
#include "custom_interfaces/msg/batteries.hpp"
#include "custom_interfaces/msg/generic_sensors.hpp"
#include "custom_interfaces/msg/gps.hpp"
#include "custom_interfaces/msg/l_path_data.hpp"
#include "custom_interfaces/msg/wind_sensors.hpp"
#include "rclcpp/node.hpp"
#include "sensors.pb.h"

namespace msg = custom_interfaces::msg;

constexpr unsigned int SATELLITE_BAUD_RATE = 19200;

/**
 * Implementation of Local Transceiver that operates through a serial interface
 */
class LocalTransceiver
{
    friend class TestLocalTransceiver_parseInMsgValid_Test;
    friend class TestLocalTransceiver_SendAndReceiveMessage;
    friend class TestLocalTransceiver_testMailboxBlackbox_Test;

public:
    /**
    * @brief Update the sensor with new GPS data
    *
    * @param gps custom_interfaces gps object
    */
    void updateSensor(msg::GPS gps);

    /**
    * @brief Update the sensor with new AIS Ships data
    *
    * @param ships custom_interfaces AISShips object
    */
    void updateSensor(msg::AISShips ships);

    /**
    * @brief Update the sensor with new Wind sensor data
    *
    * @param wind custom_interfaces WindSensors object
    */
    void updateSensor(msg::WindSensors wind);

    /**
    * @brief Update the sensor with new battery data
    *
    * @param battery custom_interfaces Batteries object
    */
    void updateSensor(msg::Batteries battery);

    /**
    * @brief Update the sensor with new generic sensor data
    *
    * @param generic custom_interfaces GenericSensors object
    */
    void updateSensor(msg::GenericSensors msg);

    /**
    * @brief Update the sensor with new local path data
    *
    * @param localData custom_interfaces LPathData object
    */
    void updateSensor(msg::LPathData localData);

    /**
    * @brief Get a copy of the sensors object
    *
    * @return Copy of sensors_
    */
    Polaris::Sensors sensors();

    /**
     * @brief Construct a new Local Transceiver object and connect it to a serial port
     *
     * @param port_name serial port (ex. /dev/ttyS0)
     * @param baud_rate baud rate of the serial port
     */
    LocalTransceiver(const std::string & port_name, uint32_t baud_rate);

    /**
     * @brief Destroy the Local Transceiver object
     *
     * @note must call stop() to properly cleanup the object
     *
     * @param sensor new sensor data
     */
    ~LocalTransceiver();

    /**
     * @brief Cleanup the Local Transceiver object by closing the serial port
     *
     * @note must be called before the object is destroyed
     *
     */
    void stop();

    /**
     * @brief Send current data to the serial port and to the remote server
     *
     * @return true  on success
     * @return false on failure
     */
    bool send();

    /**
     * @brief Send a debug command and return the output
     *
     * @param cmd string to send to the serial port
     * @return response to the sent command as a string if successful, std::nullopt on failure
     */
    std::optional<std::string> debugSend(const std::string & cmd);

    /**
     * @brief Retrieve the latest message from the remote server via the serial port
     *
     * @return The message as a binary string
     */
    custom_interfaces::msg::Path receive();

    // TEST
    bool checkMailbox();

private:
    // Serial port read/write timeout
    constexpr static const struct timeval TIMEOUT
    {
        0,        // seconds
          200000  // microseconds
    };
    // boost io service - required for boost::asio operations
    boost::asio::io_service io_;
    // serial port data where is sent and received
    boost::asio::serial_port serial_;
    // underlying sensors object
    Polaris::Sensors sensors_;

    /**
     * @brief Send a command to the serial port
     *
     * @param cmd command to send
     */
    bool send(const AT::Line & cmd);

    /**
     * @brief Read responses from serial
     *
     * @param expected_rsps expected responses
     * @return true if all expected responses are read successfully, false otherwise
     */
    bool rcvRsps(std::initializer_list<const AT::Line> expected_rsps);

    bool rcvRsp(const AT::Line & expected_rsp);

    std::optional<std::string> readRsp();

    /**
     * @brief Parse the message received from the remote server
     *
     * @param msg message received from the remote server
     * @return the data byte string payload from the message
     */
    static custom_interfaces::msg::Path parseInMsg(const std::string & msg);

    /**
     * @brief Convert a boost::asio::streambuf into a string
     * @warning Flushes the streambuf object
     *
     * @param buf streambuf to convert
     * @return buf contents as a string
     */
    static std::string streambufToStr(boost::asio::streambuf & buf);

    /**
     * @brief Compute the checksum of a binary data string.
     * The checksum is the least significant 2 bytes of the
     * sum of all bytes in the data string, where each character
     * is one byte.
     *
     * @param data data string
     * @return checksum value
     */
    static std::string checksum(const std::string & data);
};
