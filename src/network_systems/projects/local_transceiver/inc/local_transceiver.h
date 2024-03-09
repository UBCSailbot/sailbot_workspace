#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <custom_interfaces/msg/ais_ships.hpp>
#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/generic_sensors.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <custom_interfaces/msg/l_path_data.hpp>
#include <custom_interfaces/msg/wind_sensors.hpp>
#include <rclcpp/node.hpp>
#include <string>

#include "sensors.pb.h"

namespace msg = custom_interfaces::msg;

constexpr unsigned int SATELLITE_BAUD_RATE = 19200;

/**
 * Implementation of Local Transceiver that operates through a serial interface
 */
class LocalTransceiver
{
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
     * @return output of the sent cmd
     */
    std::string debugSend(const std::string & cmd);

    /**
     * @brief Retrieve the latest message from the remote server via the serial port
     *
     * @return The message as a binary string
     */
    std::string receive();

private:
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
    void send(const std::string & cmd);

    /**
     * @brief Parse the message received from the remote server
     *
     * @param msg message received from the remote server
     * @return the data byte string payload from the message
     */
    static std::string parseInMsg(const std::string & msg);

    /**
     * @brief Read a line from serial
     *
     * @return line
     */
    std::string readLine();

    /**
     * @brief Check that the last command sent to serial was valid
     *
     * @return true  if valid
     * @return false if invalid
     */
    bool checkOK();

    /**
     * @brief Compute a checksum
     *
     * @param data data string
     * @return checksum as a string
     */
    static std::string checksum(const std::string & data);
};
