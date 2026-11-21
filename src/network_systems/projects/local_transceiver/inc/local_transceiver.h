#pragma once

#include <boost/asio/streambuf.hpp>
#include <chrono>
#include <functional>
#include <optional>
#include <string>

#include "at_cmds.h"
#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#include "custom_interfaces/msg/batteries.hpp"
#include "custom_interfaces/msg/ph_sensors.hpp"
// #include "custom_interfaces/msg/pressure_sensors.hpp"
#include "custom_interfaces/msg/salinity_sensors.hpp"
#include "custom_interfaces/msg/temp_sensors.hpp"
// #include "custom_interfaces/msg/generic_sensors.hpp"
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
    friend class TestLocalTransceiver_parseInMsgInvalid_Test;
    friend class TestLocalTransceiver_SendAndReceiveMessage;
    friend class TestLocalTransceiver_testMailboxBlackbox_Test;
    friend class TestLocalTransceiver_checkCache_Test;
    friend class TestLocalTransceiver_sbdSessionRetryDelay_Test;

public:
    // Logging callback types
    using LogCallback = std::function<void(const std::string &)>;
    // Called with each valid global path received from the remote server
    using PathCallback = std::function<void(const custom_interfaces::msg::Path &)>;

    /**
     * @brief Set the callback that receives global paths. Paths can arrive from both send() and receive(), since every
     *        SBD session also checks the mailbox.
     *
     * @param path_cb Callback for received global paths
     */
    void setPathCallback(PathCallback path_cb);

    /**
    * @brief Set logging callbacks for debug and error messages
     *
     * @param debug_cb Callback for debug messages
     * @param error_cb Callback for error messages
     */
    void setLogCallbacks(LogCallback debug_cb, LogCallback error_cb);

    /**
    * @brief Update the sensor with new GPS data
    *
    * @param gps custom_interfaces gps object
    */
    void updateSensor(msg::GPS gps);

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

    // * Obsolete
    // /**
    // * @brief Update the sensor with new generic sensor data
    // *
    // * @param generic custom_interfaces GenericSensors object
    // */
    // void updateSensor(msg::GenericSensors msg);

    /**
    * @brief Update the sensor with new temperature data
    *
    * @param temperature custom_interfaces TempSensors object
    */
    void updateSensor(msg::TempSensors temperature);

    /**
    * @brief Update the sensor with new pressure data
    *
    * @param pressure custom_interfaces PressureSensors object
    */
    // void updateSensor(msg::PressureSensors pressure);

    /**
    * @brief Update the sensor with new salinity data
    *
    * @param salinity custom_interfaces SalinitySensors object
    */
    void updateSensor(msg::SalinitySensors salinity);

    /**
    * @brief Update the sensor with new pH data
    *
    * @param ph custom_interfaces PhSensors object
    */
    void updateSensor(msg::PhSensors ph);

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
     * @brief Debug helper that sends a small payload through the same
     *        flow used by `send()` but using the provided bytes. Useful for
     *        testing without serializing the full sensors protobuf.
     *        ticket #714
     *
     * @param data payload to send
     * @return true on success, false on failure
     */
    bool debugSendAT(const std::string & data);

    /**
     * @brief Checks Iridium satellite connection strength via AT+CSQ
     * @return signal strength value from 0-5, or -1 if an error occurs
     */
    int checkIridiumSignalQuality();

    /**
     * @brief Send a debug command and return the output
     *
     * @param cmd string to send to the serial port
     * @return response to the sent command as a string if successful, std::nullopt on failure
     */
    std::optional<std::string> debugSend(const std::string & cmd);

    /**
     * @brief Asynchronously save the received serialized data to a local file
     *
     * @param receivedDataBuffer string to cache
     * @return future object representing completion of the function
     */
    static void cacheGlobalWaypoints(std::string receivedDataBuffer);

    /**
     * @brief Check the Iridium mailbox and pass any received global paths to the path callback
     *
     * @param answering_ring true if this check answers a ring alert (uses AT+SBDIXA)
     * @return true if the mailbox was checked, even if it was empty
     */
    bool receive(bool answering_ring = false);

    /**
     * @brief Check the modem's ring alert flag. This only talks to the modem, so it costs nothing.
     *        The flag is set even when AT+SBDMTA ring indications are off, and stays set until a mailbox check succeeds.
     *
     * @return true if a ring alert is waiting to be answered, std::nullopt if the flag could not be read
     */
    std::optional<bool> ringAlertPending();

    /**
     * @brief Check if any SBD session (from send() or receive()) checked the mailbox recently
     *
     * @param max_age how recent the check must be
     */
    bool mailboxCheckedWithin(std::chrono::steady_clock::duration max_age) const;

    /**
     * @brief Validates the received global path waypoints before they are published to ROS
     *
     * @return true if the global path waypoints are valid, false otherwise
     */

    bool validateGlobalPathWayPoints(const custom_interfaces::msg::Path & path_to_publish);

    // TEST
    bool checkMailbox();

    /**
     * @brief Read and parse the data from the global waypoints file, if it exists
     *
     * @return The global waypoints from the cache
     */
    static std::optional<custom_interfaces::msg::Path> getCache();

private:
    // Serial port read/write timeout
    // * This 'TIMEOUT' timeout is used for the socket - it doesn't work with the actual hardware modem
    constexpr static const struct timeval TIMEOUT
    {
        0,        // seconds
          200000  // microseconds
    };
    // * This 'SERIAL_TIMEOUT' timeout actually works for the serial port, both on hardware and virtual
    static constexpr std::chrono::milliseconds SERIAL_TIMEOUT{std::chrono::seconds(15)};

    static constexpr std::chrono::seconds SMALL_WAIT{std::chrono::seconds(2)};
    static constexpr std::chrono::seconds MEDIUM_WAIT{std::chrono::seconds(2)};

    // Commands that only talk to the modem (like loading the MO buffer) rarely fail and need few retries
    static constexpr int MAX_MODEM_CMD_ATTEMPTS = 3;
    // SBD sessions go to the satellite, retried following RockBLOCK's Adaptive Retry recommendation
    // https://docs.groundcontrol.com/iot/rockblock/user-manual/adaptive-retry
    static constexpr int MAX_SBD_SESSION_ATTEMPTS = 5;
    // Each session downloads one message, so draining a queue needs one session per message. Capped so a flood of
    // messages cannot keep us in a paid loop.
    static constexpr int MAX_MAILBOX_SESSIONS = 5;

    // boost io service - required for boost::asio operations
    boost::asio::io_service io_;
    // serial port data where is sent and received
    boost::asio::serial_port serial_;
    // underlying sensors object
    Polaris::Sensors sensors_;
    // Logging callbacks
    LogCallback log_debug_;
    LogCallback log_error_;
    PathCallback on_path_;
    // When an SBD session last checked the mailbox without error
    std::optional<std::chrono::steady_clock::time_point> last_mailbox_check_;

    /**
     * @brief Send a command to the serial port
     *
     * @param cmd command to send
     */
    bool send(const AT::Line & cmd);

    /**
     * @brief Send binary data to the serial port without adding an AT command terminator
     *
     * @param data raw bytes to send
     */
    bool sendRaw(const std::string & data);

    /**
     * @brief Best-effort clear of the Iridium mobile-originated message buffer
     *
     * @return true if the modem confirms the buffer was cleared, false otherwise
     */
    bool clearMOBuffer();

    /**
     * @brief Run one SBD session (AT+SBDIX or AT+SBDIXA) and parse its result
     *
     * @param answering_ring true to use AT+SBDIXA
     * @return the session status, or std::nullopt if the result could not be read
     */
    std::optional<AT::SBDStatusRsp> runSBDSession(bool answering_ring);

    /**
     * @brief Run SBD sessions until one succeeds, retrying transient failures with a growing random delay
     *
     * @param answering_ring true to use AT+SBDIXA
     * @return the status of the successful session, or std::nullopt if none succeeded
     */
    std::optional<AT::SBDStatusRsp> runSBDSessionWithRetry(bool answering_ring);

    /**
     * @brief Handle the mailbox part of a successful SBD session: read a downloaded message, then run more sessions
     *        while the gateway says messages are queued
     *
     * @param rsp       status of the session that just finished
     * @param from_ring true if that session answered a ring alert
     * @param can_drain true if follow-up sessions are allowed. Must be false if the MO buffer might still hold a sent
     *                  message, since every session uploads the MO buffer and it would be sent (and paid for) again.
     */
    void collectMail(AT::SBDStatusRsp rsp, bool from_ring, bool can_drain);

    /**
     * @brief Read the message in the modem's MT buffer (AT+SBDRB), validate it, and cache it
     *
     * @return the global path, or std::nullopt if it could not be read or is invalid
     */
    std::optional<custom_interfaces::msg::Path> readMTMessage();

    /**
     * @brief Find the first line of an AT response that contains prefix
     */
    static std::optional<std::string> findRspLine(const std::string & rsp, const std::string & prefix);

    /**
     * @brief Random wait before an SBD session attempt: none for the first attempt, 0-5s for the next two,
     *        0-30s after that
     *
     * @param attempt zero-based attempt number
     * @return how long to wait before the attempt
     */
    static std::chrono::milliseconds sbdSessionRetryDelay(int attempt);

    /**
     * @brief Read responses from serial
     *
     * @param expected_rsps expected responses
     * @return true if all expected responses are read successfully, false otherwise
     */
    bool rcvRsps(std::initializer_list<const AT::Line> expected_rsps);

    bool rcvRsp(const AT::Line & expected_rsp);

    /**
     * @brief Read until the command's final result line ("OK" or "ERROR")
     *
     * @return everything read, or std::nullopt on timeout. Callers must check for the line they expect.
     */
    std::optional<std::string> readRsp();

    /**
     * @brief Drain/clear any pending data in the serial buffer to remove stale data
     *        from previous iterations or operations. This helps prevent stale data from one attempt/operation
     *        from interfering with subsequent reads.
     */
    void clearSerialBuffer();

    /**
     * @brief  Read a binary response from Iridium
     *
     * @return std::optional<std::string>
     */
    std::optional<std::string> readBinaryRsp();

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

    template <typename AsyncReadOp>
    bool runWithTimeout(AsyncReadOp && op, boost::system::error_code & out_ec);
};
