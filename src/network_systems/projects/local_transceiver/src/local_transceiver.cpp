#include "local_transceiver.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>
#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/generic_sensors.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <custom_interfaces/msg/l_path_data.hpp>
#include <custom_interfaces/msg/wind_sensors.hpp>
#include <exception>
#include <random>
#include <regex>
#include <stdexcept>
#include <string>

#include "at_cmds.h"
#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "filesystem"
#include "fstream"
#include "global_path.pb.h"
#include "sensors.pb.h"
#include "waypoint.pb.h"

using boost::system::error_code;
using Polaris::Sensors;
namespace bio = boost::asio;

constexpr float LAT_LBOUND = -90.0F;
constexpr float LAT_UBOUND = 90.0F;
constexpr float LON_LBOUND = -180.0F;
constexpr float LON_UBOUND = 180.0F;

namespace
{
/**
 * @brief Tells bio::async_read_until to stop at the end of an AT command's final result, "OK" or "ERROR".
 *        Stopping only at "OK" would make every command the modem rejects wait for the full SERIAL_TIMEOUT.
 */
struct FinalResultMatcher
{
    template <typename Iterator>
    std::pair<Iterator, bool> operator()(Iterator begin, Iterator end) const
    {
        const std::string data(begin, end);
        for (const std::string & result : {AT::STATUS_OK + AT::DELIMITER, AT::STATUS_ERROR + AT::DELIMITER}) {
            size_t pos = data.find(result);
            if (pos != std::string::npos) {
                return {begin + static_cast<std::ptrdiff_t>(pos + result.size()), true};
            }
        }
        return {end, false};  // keep reading
    }
};
}  // namespace

template <>
struct boost::asio::is_match_condition<FinalResultMatcher> : public std::true_type
{
};

void LocalTransceiver::updateSensor(msg::GPS gps)
{
    sensors_.mutable_gps()->set_heading(gps.heading.heading);
    sensors_.mutable_gps()->set_latitude(gps.lat_lon.latitude);
    sensors_.mutable_gps()->set_longitude(gps.lat_lon.longitude);
    sensors_.mutable_gps()->set_speed(gps.speed.speed);
}

void LocalTransceiver::updateSensor(msg::WindSensors wind)
{
    sensors_.clear_wind_sensors();
    for (const msg::WindSensor & wind_data : wind.wind_sensors) {
        Sensors::Wind * new_wind = sensors_.add_wind_sensors();
        new_wind->set_direction(wind_data.direction);
        new_wind->set_speed(wind_data.speed.speed);
    }
}

void LocalTransceiver::updateSensor(msg::Batteries battery)
{
    sensors_.clear_batteries();
    for (const msg::HelperBattery & battery_info : battery.batteries) {
        Sensors::Battery * new_battery = sensors_.add_batteries();
        new_battery->set_current(battery_info.current);
        new_battery->set_voltage(battery_info.voltage);
    }
}

// void LocalTransceiver::updateSensor(msg::GenericSensors msg)
// {
//     sensors_.clear_data_sensors();
//     for (const msg::HelperGenericSensor & sensors_data : msg.generic_sensors) {
//         Sensors::Generic * new_sensor = sensors_.add_data_sensors();
//         new_sensor->set_data(sensors_data.data);
//         new_sensor->set_id(sensors_data.id);
//     }
// }

void LocalTransceiver::updateSensor(msg::TempSensors temperature)
{
    sensors_.clear_temp_sensors();
    for (const msg::TempSensor & temperature_data : temperature.temp_sensors) {
        sensors_.add_temp_sensors(temperature_data.temp.temp);
    }
}

void LocalTransceiver::updateSensor(msg::PhSensors ph)
{
    sensors_.clear_ph_sensors();
    for (const msg::PhSensor & ph_data : ph.ph_sensors) {
        sensors_.add_ph_sensors(ph_data.ph.ph);
    }
}

void LocalTransceiver::updateSensor(msg::SalinitySensors salinity)
{
    sensors_.clear_salinity_sensors();
    for (const msg::SalinitySensor & salinity_data : salinity.salinity_sensors) {
        sensors_.add_salinity_sensors(salinity_data.salinity.salinity);
    }
}

// Pressure sensor retired (removed from CAN and no longer sent). Kept commented so the last
// compile-time dependency on the Sensors proto's pressure_sensors field is gone.
// void LocalTransceiver::updateSensor(msg::PressureSensors pressure)
// {
//     sensors_.clear_pressure_sensors();
//     for (const msg::PressureSensor & pressure_data : pressure.pressure_sensors) {
//         sensors_.add_pressure_sensors(pressure_data.pressure.pressure);
//     }
// }

void LocalTransceiver::updateSensor(msg::LPathData localData)
{
    sensors_.clear_local_path_data();
    Sensors::Path * new_local = sensors_.mutable_local_path_data();
    for (const msg::HelperLatLon & local_data : localData.local_path.waypoints) {
        Polaris::Waypoint * waypoint = new_local->add_waypoints();
        waypoint->set_latitude(local_data.latitude);
        waypoint->set_longitude(local_data.longitude);
    }
}

Sensors LocalTransceiver::sensors() { return sensors_; }

void LocalTransceiver::setLogCallbacks(LogCallback debug_cb, LogCallback error_cb)
{
    log_debug_ = debug_cb;
    log_error_ = error_cb;
}

void LocalTransceiver::setPathCallback(PathCallback path_cb) { on_path_ = path_cb; }

LocalTransceiver::LocalTransceiver(const std::string & port_name, const uint32_t baud_rate) : serial_(io_, port_name)
{
    serial_.set_option(bio::serial_port_base::baud_rate(baud_rate));
    // Set a timeout for read/write operations on the serial port
    setsockopt(serial_.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &TIMEOUT, sizeof(TIMEOUT));
    std::ofstream cacheFile(CACHE_PATH);
    cacheFile.close();
};

LocalTransceiver::~LocalTransceiver()
{
    // Intentionally left blank
}

void LocalTransceiver::stop()
{
    serial_.cancel();
    serial_.close();  // Can throw an exception so cannot be put in the destructor
}

bool LocalTransceiver::send()
{
    std::string data;
    // Make sure to get a copy of the sensors because repeated calls may give us different results
    Polaris::Sensors sensors = sensors_;

    if (!sensors.SerializeToString(&data)) {
        if (log_error_) {
            log_error_("Failed to serialize sensors string\n" + sensors.DebugString());
        }
        return false;
    }
    if (data.size() >= MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES) {
        // if this proves to be a problem, we need a solution to split the data into multiple messages
        std::string err_string =
          "Data too large!\n"
          "Attempted: " +
          std::to_string(data.size()) + " bytes\n" + sensors.DebugString() +
          "\n"
          "No implementation to handle this!";
        throw std::length_error(err_string);
    }
    std::cout << "Sending " + std::to_string(data.size()) + " bytes\n";

    if (log_debug_) {
        log_debug_("send() starting...");
    }

    std::string write_bin_cmd_str = AT::write_bin::CMD + std::to_string(data.size());  //according to specs
    AT::Line    at_write_cmd(write_bin_cmd_str);
    std::string msg_str = data + checksum(data);

    // Load the message into the modem's MO buffer. It stays there between SBD sessions, so it only needs to be
    // loaded once no matter how many sessions are attempted.
    bool loaded = false;
    for (int i = 0; i < MAX_MODEM_CMD_ATTEMPTS && !loaded; i++) {
        std::this_thread::sleep_for(SMALL_WAIT);
        clearSerialBuffer();  // Clear any stale data from previous iteration

        loaded = send(at_write_cmd) &&
                 rcvRsps({
                   at_write_cmd,
                   AT::Line(AT::DELIMITER),
                   AT::Line(AT::RSP_READY),
                   AT::Line("\n"),
                 }) &&
                 sendRaw(msg_str) &&
                 rcvRsps({
                   AT::Line(AT::DELIMITER),
                   AT::Line(AT::write_bin::rsp::SUCCESS),
                   AT::Line("\n"),
                   AT::Line(AT::DELIMITER),
                   AT::Line(AT::STATUS_OK),
                   AT::Line("\n"),
                 });
    }
    if (!loaded) {
        if (log_error_) {
            log_error_("Failed to load message into the Iridium MO buffer");
        }
        return false;
    }

    std::optional<AT::SBDStatusRsp> rsp = runSBDSessionWithRetry(false);
    if (!rsp) {
        if (log_error_) {
            log_error_("Failed to transmit data to satellite\n" + sensors.DebugString());
        }
        return false;
    }

    // The session also checked the mailbox. Clear the MO buffer first so follow-up sessions to collect more mail
    // cannot send this data again.
    bool mo_cleared = clearMOBuffer();
    collectMail(*rsp, false, mo_cleared);
    return true;
}

std::optional<AT::SBDStatusRsp> LocalTransceiver::runSBDSessionWithRetry(bool answering_ring)
{
    // Only the SBD session goes to the satellite, and it is where failures happen (signal drops, no network).
    // Failed sessions are not charged on RockBLOCK, so transient failures are retried with a growing random delay.
    for (int attempt = 0; attempt < MAX_SBD_SESSION_ATTEMPTS; attempt++) {
        std::this_thread::sleep_for(sbdSessionRetryDelay(attempt));

        // CSQ 1 is the minimum needed to transmit, so only skip the session when there is no signal at all.
        // If the signal quality could not be read (-1), try anyway and let the MO status decide.
        int signal_quality = checkIridiumSignalQuality();  // also clears the serial buffer
        if (signal_quality == AT::signal_quality::NOSIGNAL) {
            if (log_debug_) {
                log_debug_("No Iridium signal, skipping SBD session attempt " + std::to_string(attempt));
            }
            continue;
        }

        // NEEDS AN ACTIVE SERVER ON $WEBHOOK_SERVER_ENDPOINT OR VIRTUAL IRIDIUM WILL CRASH
        std::optional<AT::SBDStatusRsp> rsp = runSBDSession(answering_ring);
        if (!rsp) {
            // The message may have been delivered even though the result could not be read. Retrying could send
            // (and pay for) a duplicate, so give up until the next send.
            if (log_error_) {
                log_error_("Could not read SBD session result, not retrying");
            }
            return std::nullopt;
        }

        const std::string mo_status_str = std::to_string(rsp->MO_status_);
        switch (rsp->classifyMO()) {
            case AT::MOResult::SUCCESS:
                if (log_debug_) {
                    log_debug_("SBD session succeeded on attempt " + std::to_string(attempt));
                }
                return rsp;
            case AT::MOResult::RETRY_SOON:
                if (log_error_) {
                    log_error_(
                      "SBD session attempt " + std::to_string(attempt) + " failed with MO status " + mo_status_str +
                      ", retrying");
                }
                break;
            case AT::MOResult::RETRY_LATER:
                if (log_error_) {
                    log_error_(
                      "Iridium gateway asked to retry later (MO status " + mo_status_str +
                      "), not retrying until the next attempt");
                }
                return std::nullopt;
            case AT::MOResult::FATAL:
                if (log_error_) {
                    log_error_(
                      "SBD session failed with MO status " + mo_status_str + ", which retrying cannot fix");
                }
                return std::nullopt;
        }
    }

    if (log_error_) {
        log_error_("SBD session failed after " + std::to_string(MAX_SBD_SESSION_ATTEMPTS) + " attempts");
    }
    return std::nullopt;
}

std::optional<AT::SBDStatusRsp> LocalTransceiver::runSBDSession(bool answering_ring)
{
    static const AT::Line sbdix_cmd  = AT::Line(AT::SBD_SESSION);
    static const AT::Line sbdixa_cmd = AT::Line(AT::SBD_SESSION_RING);
    const AT::Line &      cmd        = answering_ring ? sbdixa_cmd : sbdix_cmd;
    if (!send(cmd)) {
        return std::nullopt;
    }

    if (!rcvRsps({
          cmd,
          AT::Line(AT::DELIMITER),
        })) {
        return std::nullopt;
    }

    auto opt_rsp = readRsp();
    if (!opt_rsp) {
        return std::nullopt;
    }

    // The response looks like "+SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>\r\n\r\nOK\r"
    // (SBDIXA replies with the same "+SBDIX:" line)
    std::optional<std::string> line = findRspLine(opt_rsp.value(), "SBDIX:");
    if (!line) {
        if (log_error_) {
            log_error_("No SBDIX status in response: \"" + opt_rsp.value() + "\"");
        }
        return std::nullopt;
    }
    try {
        return AT::SBDStatusRsp(*line);
    } catch (const std::exception & e) {
        if (log_error_) {
            log_error_(std::string("Failed to parse SBD session result: ") + e.what());
        }
        return std::nullopt;
    }
}

std::optional<std::string> LocalTransceiver::findRspLine(const std::string & rsp, const std::string & prefix)
{
    // Search for the line instead of assuming it comes first, since stale bytes can precede it
    std::vector<std::string> lines;
    boost::algorithm::split(lines, rsp, boost::is_any_of(AT::DELIMITER));
    for (const std::string & line : lines) {
        if (line.find(prefix) != std::string::npos) {
            return line;
        }
    }
    return std::nullopt;
}

void LocalTransceiver::collectMail(AT::SBDStatusRsp rsp, bool from_ring, bool can_drain)
{
    for (int session = 0; session < MAX_MAILBOX_SESSIONS; session++) {
        if (!rsp.mailboxChecked()) {
            if (log_error_) {
                log_error_("Iridium gateway could not check the mailbox (MT status 2)");
            }
            return;
        }
        last_mailbox_check_ = std::chrono::steady_clock::now();

        if (rsp.MTReceived()) {
            // Only the first session can tell us this: follow-up sessions are expected to find mail
            if (session == 0 && !from_ring && log_debug_) {
                log_debug_("Message collected by a regular SBD session before a ring alert was answered");
            }
            // Must be read now, the next session overwrites the modem's MT buffer
            std::optional<custom_interfaces::msg::Path> path = readMTMessage();
            if (path && on_path_) {
                on_path_(*path);
            }
        }

        if (rsp.MT_queued_ == 0) {
            return;
        }
        if (!can_drain) {
            if (log_error_) {
                log_error_(
                  std::to_string(rsp.MT_queued_) +
                  " messages still queued, but the MO buffer could not be cleared so they are left for later");
            }
            return;
        }
        if (log_debug_) {
            log_debug_(std::to_string(rsp.MT_queued_) + " more messages queued, collecting the next one");
        }
        std::optional<AT::SBDStatusRsp> next = runSBDSessionWithRetry(false);
        if (!next) {
            return;
        }
        rsp = *next;
    }

    if (log_error_) {
        log_error_("Stopped collecting mail after " + std::to_string(MAX_MAILBOX_SESSIONS) + " sessions");
    }
}

bool LocalTransceiver::mailboxCheckedWithin(std::chrono::steady_clock::duration max_age) const
{
    return last_mailbox_check_ && std::chrono::steady_clock::now() - *last_mailbox_check_ < max_age;
}

std::optional<bool> LocalTransceiver::ringAlertPending()
{
    static const AT::Line sbdsx_cmd = AT::Line(AT::SBD_STATUS_EXT);
    clearSerialBuffer();
    if (!send(sbdsx_cmd) || !rcvRsps({sbdsx_cmd, AT::Line(AT::DELIMITER)})) {
        return std::nullopt;
    }

    auto opt_rsp = readRsp();
    if (!opt_rsp) {
        return std::nullopt;
    }

    // The response looks like "+SBDSX: <MO flag>, <MOMSN>, <MT flag>, <MTMSN>, <RA flag>, <msg waiting>\r\n\r\nOK\r"
    std::optional<std::string> line = findRspLine(opt_rsp.value(), "SBDSX:");
    if (!line) {
        if (log_error_) {
            log_error_("No SBDSX status in response: \"" + opt_rsp.value() + "\"");
        }
        return std::nullopt;
    }
    try {
        return AT::SBDStatusExtRsp(*line).RA_flag_;
    } catch (const std::exception & e) {
        if (log_error_) {
            log_error_(std::string("Failed to parse SBDSX status: ") + e.what());
        }
        return std::nullopt;
    }
}

std::chrono::milliseconds LocalTransceiver::sbdSessionRetryDelay(int attempt)
{
    static constexpr int                       NUM_SHORT_RETRIES = 2;
    static constexpr std::chrono::milliseconds SHORT_MAX_DELAY{std::chrono::seconds(5)};
    static constexpr std::chrono::milliseconds LONG_MAX_DELAY{std::chrono::seconds(30)};

    if (attempt <= 0) {
        return std::chrono::milliseconds(0);
    }
    const std::chrono::milliseconds max_delay = attempt <= NUM_SHORT_RETRIES ? SHORT_MAX_DELAY : LONG_MAX_DELAY;

    // Random delays avoid retrying in lockstep with whatever caused the failure
    static thread_local std::mt19937                     rng{std::random_device{}()};
    std::uniform_int_distribution<std::chrono::milliseconds::rep> dist(0, max_delay.count());
    return std::chrono::milliseconds(dist(rng));
}

bool LocalTransceiver::debugSendAT(const std::string & data)
{
    if (data.size() >= MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES) {
        if (log_error_) {
            log_error_(
              "Debug message/data too large: " + std::to_string(data.size()) + " bytes, but with a limit of " +
              std::to_string(MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES) + " bytes");
        }
        return false;
    }

    if (log_debug_) {
        log_debug_("Debug: Sending " + std::to_string(data.size()) + " bytes via debugSendAT");
    }

    std::string write_bin_cmd_str = AT::write_bin::CMD + std::to_string(data.size());
    AT::Line    at_write_cmd(write_bin_cmd_str);

    static constexpr int MAX_NUM_RETRIES = 20;
    for (int i = 0; i < MAX_NUM_RETRIES; i++) {
        if (log_debug_) {
            log_debug_("Debug: clearing buffer (attempt " + std::to_string(i) + ")");
        }

        std::this_thread::sleep_for(std::chrono::seconds(SMALL_WAIT));
        clearSerialBuffer();  // Clear any stale data from previous iteration

        std::this_thread::sleep_for(std::chrono::seconds(MEDIUM_WAIT));
        if (log_debug_) {
            log_debug_("Debug: cleared buffer, checking Iridium signal quality (attempt " + std::to_string(i) + ")");
        }

        // int current_iridium_signal_quality = 5;
        int current_iridium_signal_quality = checkIridiumSignalQuality();
        if (current_iridium_signal_quality == -1) {
            if (log_error_) {
                log_error_("Debug: Failed to check Iridium signal quality");
            }
            continue;
        }
        if (current_iridium_signal_quality < AT::signal_quality::GOOD) {
            if (log_debug_) {
                log_debug_(
                  "Debug: Iridium signal quality is currently " + std::to_string(current_iridium_signal_quality) +
                  ", retrying...");
            }
            std::this_thread::sleep_for(
              std::chrono::seconds(MEDIUM_WAIT));  // Wait a moment before retrying if no signal

            continue;
        }
        if (log_debug_) {
            log_debug_(
              "Debug: Iridium signal quality is " + std::to_string(current_iridium_signal_quality) +
              ", proceeding to send write command (attempt " + std::to_string(i) + ")");
        }

        if (!send(at_write_cmd)) {
            if (log_error_) {
                log_error_("Debug: failed to send write command (attempt " + std::to_string(i) + ")");
            }
            continue;
        }
        if (log_debug_) {
            log_debug_("Debug: sent write command (attempt " + std::to_string(i) + "): " + at_write_cmd.str_);
        }

        if (!rcvRsps({
              at_write_cmd,
              AT::Line(AT::DELIMITER),
              AT::Line(AT::RSP_READY),
              AT::Line("\n"),
            })) {
            if (log_error_) {
                log_error_("Debug: did not receive ready prompt (attempt " + std::to_string(i) + ")");
            }
            continue;
        }
        if (log_debug_) {
            log_debug_("Debug: received ready prompt (attempt " + std::to_string(i) + ")");
        }

        std::string msg_str = data + checksum(data);
        if (!sendRaw(msg_str)) {
            if (log_error_) {
                log_error_("Debug: failed to send payload (attempt " + std::to_string(i) + ")");
            }
            continue;
        }
        if (log_debug_) {
            log_debug_(
              "Debug: sent " + std::to_string(data.size()) + " payload bytes plus checksum (attempt " +
              std::to_string(i) + ")");
        }

        if (!rcvRsps({
              AT::Line(AT::DELIMITER),
              AT::Line(AT::write_bin::rsp::SUCCESS),
              AT::Line("\n"),
              AT::Line(AT::DELIMITER),
              AT::Line(AT::STATUS_OK),
              AT::Line("\n"),
            })) {
            if (log_error_) {
                log_error_("Debug: write did not complete successfully (attempt " + std::to_string(i) + ")");
            }
            continue;
        }
        if (log_debug_) {
            log_debug_("Debug: write completed successfully (attempt " + std::to_string(i) + ")");
        }

        clearSerialBuffer();

        static const AT::Line sbdix_cmd = AT::Line(AT::SBD_SESSION);
        if (!send(sbdix_cmd)) {
            if (log_error_) {
                log_error_("Debug: failed to send SBDIX command (attempt " + std::to_string(i) + ")");
            }
            return false;
        }
        if (log_debug_) {
            log_debug_("Debug: sent SBDIX command (attempt " + std::to_string(i) + "): " + sbdix_cmd.str_);
        }

        if (!rcvRsps({
              sbdix_cmd,
              AT::Line(AT::DELIMITER),
            })) {
            if (log_error_) {
                log_error_("Debug: did not receive SBDIX response header (attempt " + std::to_string(i) + ")");
            }
            return false;
        }
        if (log_debug_) {
            log_debug_("Debug: received SBDIX response header (attempt " + std::to_string(i) + ")");
        }

        auto opt_rsp = readRsp();
        if (!opt_rsp) {
            if (log_error_) {
                log_error_("Debug: readRsp returned no response (attempt " + std::to_string(i) + ")");
            }
            return false;
        }
        if (log_debug_) {
            log_debug_("Debug: readRsp received response (attempt " + std::to_string(i) + "): " + opt_rsp.value());
        }

        std::string              opt_rsp_val = opt_rsp.value();
        std::vector<std::string> sbd_status_vec;
        boost::algorithm::split(sbd_status_vec, opt_rsp_val, boost::is_any_of(AT::DELIMITER));

        AT::SBDStatusRsp rsp(sbd_status_vec[0]);

        if (log_debug_) {
            log_debug_("SBDIX parsed rsp MO Status: " + std::to_string(rsp.MO_status_));
        }

        if (rsp.MOSuccess()) {
            if (log_debug_) {
                log_debug_("Debug: debugSendAT transmitted successfully");
            }
            clearMOBuffer();
            return true;
        }
        if (log_error_) {
            log_error_("Debug: MO Status: " + std::to_string(rsp.MO_status_));
        }
        return false;
    }

    if (log_error_) {
        log_error_("Debug: Failed to transmit debug payload to satellite");
    }
    return false;
}

std::optional<std::string> LocalTransceiver::debugSend(const std::string & cmd)
{
    AT::Line    at_cmd(cmd);
    std::string sent_cmd;

    if (!send(at_cmd)) {
        return std::nullopt;
    }

    return readRsp();
}

void LocalTransceiver::cacheGlobalWaypoints(std::string receivedDataBuffer)
{
    std::filesystem::path cache{CACHE_PATH};
    std::filesystem::path cache_temp{CACHE_TEMP_PATH};
    std::ofstream         writeFile(CACHE_TEMP_PATH, std::ios::binary);
    if (!writeFile) {
        // Note: This is a static method, so no logging callback available
        std::cerr << "Failed to create temp cache file" << std::endl;
    }
    writeFile.write(receivedDataBuffer.data(), static_cast<std::streamsize>(receivedDataBuffer.size()));
    writeFile.close();
    std::filesystem::rename(CACHE_TEMP_PATH, CACHE_PATH);
}

bool LocalTransceiver::receive(bool answering_ring)
{
    static const AT::Line check_conn_cmd       = AT::Line(AT::CHECK_CONN);
    static const AT::Line disable_ctrlflow_cmd = AT::Line(AT::DSBL_CTRLFLOW);

    // Local setup, no satellite cost. The MO buffer must be empty, or the session would upload (and pay for)
    // whatever a failed send() left in it.
    bool ready = false;
    for (int i = 0; i < MAX_MODEM_CMD_ATTEMPTS && !ready; i++) {
        std::this_thread::sleep_for(SMALL_WAIT);
        clearSerialBuffer();  // Clear any stale data from previous iteration

        ready = send(check_conn_cmd) &&
                rcvRsps({check_conn_cmd, AT::Line(AT::DELIMITER), AT::Line(AT::STATUS_OK), AT::Line("\n")}) &&
                send(disable_ctrlflow_cmd) &&
                rcvRsps({disable_ctrlflow_cmd, AT::Line(AT::DELIMITER), AT::Line(AT::STATUS_OK), AT::Line("\n")}) &&
                clearMOBuffer();
    }
    if (!ready) {
        if (log_error_) {
            log_error_("Iridium modem not ready for a mailbox check");
        }
        return false;
    }

    std::optional<AT::SBDStatusRsp> rsp = runSBDSessionWithRetry(answering_ring);
    if (!rsp) {
        return false;
    }
    collectMail(*rsp, answering_ring, true);
    return rsp->mailboxChecked();
}

std::optional<custom_interfaces::msg::Path> LocalTransceiver::readMTMessage()
{
    static constexpr int  MAX_READ_ATTEMPTS    = 5;
    static const AT::Line message_to_queue_cmd = AT::Line(AT::DNLD_TO_QUEUE);

    std::optional<std::string> payload;
    for (int i = 0; i < MAX_READ_ATTEMPTS && !payload; i++) {
        if (i > 0) {
            // A failed read can leave partial payload bytes behind, which would break the next attempt's echo check
            clearSerialBuffer();
        }
        if (send(message_to_queue_cmd) && rcvRsps({message_to_queue_cmd})) {
            payload = readBinaryRsp();
        }
    }
    if (!payload) {
        // The gateway deleted its copy when it was downloaded, so this message is gone
        if (log_error_) {
            log_error_("Failed to read a received message from the Iridium MT buffer, it is lost");
        }
        return std::nullopt;
    }
    if (log_debug_) {
        log_debug_("Read " + std::to_string(payload->size()) + " byte message from the Iridium MT buffer");
    }

    custom_interfaces::msg::Path path = parseInMsg(*payload);
    if (!validateGlobalPathWayPoints(path)) {
        return std::nullopt;
    }
    // Only cache valid paths: getCache() publishes the cache on startup without validating it
    cacheGlobalWaypoints(*payload);
    return path;
}

bool LocalTransceiver::validateGlobalPathWayPoints(const custom_interfaces::msg::Path & path_to_publish)
{
    if (path_to_publish.waypoints.empty()) {
        return false;
    }

    for (const auto & waypoint : path_to_publish.waypoints) {  //NOLINT
        if (
          (waypoint.latitude < LAT_LBOUND || waypoint.latitude > LAT_UBOUND) ||
          (waypoint.longitude < LON_LBOUND || waypoint.longitude > LON_UBOUND)) {
            if (log_error_) {
                log_error_(
                  "Invalid waypoint lat/lon received: (" + std::to_string(waypoint.latitude) + ", " +
                  std::to_string(waypoint.longitude) + "). Publishing empty path.");
            }
            return false;
        }
    }

    return true;
}

bool LocalTransceiver::send(const AT::Line & cmd)
{
    boost::system::error_code ec;
    bio::write(serial_, bio::buffer(cmd.str_, cmd.str_.size()), ec);
    if (ec) {
        std::cerr << "Write failed with error: " << ec.message() << std::endl;
        return false;
    }
    return true;
}

bool LocalTransceiver::sendRaw(const std::string & data)
{
    boost::system::error_code ec;
    bio::write(serial_, bio::buffer(data.data(), data.size()), ec);
    if (ec) {
        std::cerr << "Binary write failed with error: " << ec.message() << std::endl;
        return false;
    }
    return true;
}

bool LocalTransceiver::clearMOBuffer()
{
    static const AT::Line clear_mo_cmd = AT::Line(AT::CLEAR_MO_BUFFER);
    if (!send(clear_mo_cmd)) {
        if (log_error_) {
            log_error_("Failed to send command to clear the Iridium MO buffer");
        }
        return false;
    }

    if (!rcvRsps({
          clear_mo_cmd,
          AT::Line(AT::DELIMITER),
          AT::Line("0"),
          AT::Line("\n"),
          AT::Line(AT::DELIMITER),
          AT::Line(AT::STATUS_OK),
          AT::Line("\n"),
        })) {
        if (log_error_) {
            log_error_("Iridium modem did not confirm that the MO buffer was cleared");
        }
        return false;
    }

    if (log_debug_) {
        log_debug_("Iridium MO buffer cleared successfully");
    }
    return true;
}

custom_interfaces::msg::Path LocalTransceiver::parseInMsg(const std::string & msg)
{
    Polaris::GlobalPath path;
    path.ParseFromString(msg);

    custom_interfaces::msg::Path                      soln;
    std::vector<custom_interfaces::msg::HelperLatLon> waypoints;

    for (auto waypoint : path.waypoints()) {
        custom_interfaces::msg::HelperLatLon helperLatLon;
        helperLatLon.set__longitude(waypoint.longitude());
        helperLatLon.set__latitude(waypoint.latitude());

        waypoints.push_back(helperLatLon);
    }

    soln.set__waypoints(waypoints);
    return soln;
}

std::optional<custom_interfaces::msg::Path> LocalTransceiver::getCache()
{
    if (std::filesystem::exists(CACHE_PATH) && std::filesystem::file_size(CACHE_PATH) > 0) {
        std::ifstream                input(CACHE_PATH, std::ios::binary);
        std::string                  cachedDataBuffer(std::istreambuf_iterator<char>(input), {});
        custom_interfaces::msg::Path to_publish = parseInMsg(cachedDataBuffer);
        return std::make_optional(to_publish);
    }
    return std::nullopt;
}

bool LocalTransceiver::rcvRsp(const AT::Line & expected_rsp)
{
    bio::streambuf            buf;
    boost::system::error_code ec;

    bool success = runWithTimeout(
      [&](auto handler) { bio::async_read(serial_, buf, bio::transfer_exactly(expected_rsp.str_.size()), handler); },
      ec);

    if (!success) {
        if (log_error_) {
            log_error_("rcvRsp timeout or error: " + ec.message());
        }
        return false;
    }

    std::string outstr = streambufToStr(buf);

    if (outstr != expected_rsp.str_) {
        if (log_error_) {
            log_error_("Expected: \"" + expected_rsp.str_ + "\" but got: \"" + outstr + "\"");
        }
        return false;
    }

    return true;
}

bool LocalTransceiver::rcvRsps(std::initializer_list<const AT::Line> expected_rsps)
{
    // All responses must match the expected responses
    return std::all_of(
      expected_rsps.begin(), expected_rsps.end(), [this](const AT::Line & e_rsp) { return rcvRsp(e_rsp); });
}

std::optional<std::string> LocalTransceiver::readBinaryRsp()
{
    std::array<uint8_t, 2>    len_buf;
    boost::system::error_code ec;

    bool success = runWithTimeout(
      [&](auto handler) { bio::async_read(serial_, bio::buffer(len_buf), bio::transfer_exactly(2), handler); }, ec);

    if (!success) {
        if (log_error_) {
            log_error_("readBinaryRsp: failed to read length prefix: " + ec.message());
        }
        return std::nullopt;
    }

    uint16_t msg_len = (static_cast<uint16_t>(len_buf[0]) << 8) | static_cast<uint16_t>(len_buf[1]);  //NOLINT

    if (msg_len == 0) {
        if (log_error_) {
            log_error_("readBinaryRsp: zero length message");
        }
        return std::nullopt;
    }

    std::string payload(msg_len, '\0');
    success = runWithTimeout(
      [&](auto handler) { bio::async_read(serial_, bio::buffer(payload), bio::transfer_exactly(msg_len), handler); },
      ec);

    if (!success) {
        if (log_error_) {
            log_error_("readBinaryRsp: failed to read payload: " + ec.message());
        }
        return std::nullopt;
    }

    std::array<uint8_t, 2> chk_buf;
    success = runWithTimeout(
      [&](auto handler) { bio::async_read(serial_, bio::buffer(chk_buf), bio::transfer_exactly(2), handler); }, ec);

    if (!success) {
        if (log_error_) {
            log_error_("readBinaryRsp: failed to read checksum: " + ec.message());
        }
        return std::nullopt;
    }

    uint16_t received_checksum = (static_cast<uint16_t>(chk_buf[0]) << 8) | static_cast<uint16_t>(chk_buf[1]);  //NOLINT
    uint16_t calculated_checksum = 0;
    for (unsigned char byte : payload) {
        calculated_checksum += byte;
    }

    if (received_checksum != calculated_checksum) {
        if (log_error_) {
            log_error_("readBinaryRsp: checksum mismatch");
        }
        return std::nullopt;
    }

    return payload;
}

std::optional<std::string> LocalTransceiver::readRsp()
{
    bio::streambuf            buf;
    boost::system::error_code ec;

    bool success = runWithTimeout(
      [&](auto handler) { bio::async_read_until(serial_, buf, FinalResultMatcher(), handler); }, ec);

    if (!success) {
        if (log_error_) {
            log_error_("readRsp timeout or error: " + ec.message());
        }
        return std::nullopt;
    }

    std::string rsp_str = streambufToStr(buf);
    if (!rsp_str.empty()) {
        rsp_str.pop_back();  // remove trailing '\n'
    }

    return rsp_str;
}

std::string LocalTransceiver::checksum(const std::string & data)
{
    uint16_t sum = 0;
    for (char c : data) {
        sum += static_cast<uint8_t>(c);
    }

    char checksum_low  = static_cast<char>(sum & 0xff);           // NOLINT(readability-magic-numbers)
    char checksum_high = static_cast<char>((sum & 0xff00) >> 8);  // NOLINT(readability-magic-numbers)

    return std::string{checksum_high, checksum_low};
}

std::string LocalTransceiver::streambufToStr(bio::streambuf & buf)
{
    std::string str = std::string(
      bio::buffers_begin(buf.data()), bio::buffers_begin(buf.data()) + static_cast<int64_t>(buf.data().size()));
    buf.consume(buf.size());
    return str;
}

void LocalTransceiver::clearSerialBuffer()
{
    int fd        = serial_.lowest_layer().native_handle();
    int old_flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, old_flags | O_NONBLOCK);

    const std::size_t SERIAL_BUFFER_SIZE = 1024;
    std::vector<char> buf(SERIAL_BUFFER_SIZE);

    while (true) {
        ssize_t bytes_read = ::read(fd, buf.data(), buf.size());
        if (bytes_read > 0) {
            if (log_debug_) {
                log_debug_("Cleared " + std::to_string(bytes_read) + " bytes from serial buffer.");
            }
            continue;
        }
        if (bytes_read == 0) {
            if (log_debug_) {
                log_debug_("Serial buffer cleared successfully.");
            }
            break;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            if (log_debug_) {
                log_debug_("No more data to read from serial buffer.");
            }
            break;
        }
        if (log_error_) {
            log_error_("Failed to read from serial port with error: " + std::to_string(errno));
        }
        break;
    }

    fcntl(fd, F_SETFL, old_flags);  // Restore original flags
}

int LocalTransceiver::checkIridiumSignalQuality()
{
    // Assumes AT is already initialized and ready

    if (log_debug_) {
        log_debug_("Checking signal quality...");
    }

    std::this_thread::sleep_for(std::chrono::seconds(SMALL_WAIT));

    clearSerialBuffer();
    static const AT::Line at_check_conn_strength_cmd = AT::Line(AT::CHECK_SIG_QUALITY);

    std::this_thread::sleep_for(std::chrono::seconds(SMALL_WAIT));

    if (!send(at_check_conn_strength_cmd)) {
        if (log_error_) {
            log_error_("Debug: failed to send CSQ command");
            return -1;
        }
    }
    if (log_debug_) {
        log_debug_("Debug: sent CSQ command");
    }

    if (!rcvRsps({
          at_check_conn_strength_cmd,
          AT::Line(AT::DELIMITER),
        })) {
        if (log_error_) {
            log_error_("Debug: did not receive CSQ response header");
            return -1;
        }
    }
    if (log_debug_) {
        log_debug_("Debug: received CSQ response header");
    }

    auto opt_rsp = readRsp();
    if (!opt_rsp) {
        if (log_error_) {
            log_error_("Debug: readRsp returned no response");
            return -1;
        }
    }
    if (log_debug_) {
        log_debug_("Debug: readRsp received response");
    }

    // This string will look something like:
    // "+CSQ:<Signal Quality>\r\n\r\nOK\r"
    // on success
    std::string opt_rsp_val = opt_rsp.value();
    // std::vector<std::string> sbd_status_vec;
    // boost::algorithm::split(sbd_status_vec, opt_rsp_val, boost::is_any_of(AT::DELIMITER));

    // AT::SBDStatusRsp rsp(sbd_status_vec[0]);
    // if (rsp.MOSuccess()) {
    //     return true;
    // }
    int signal_quality =
      opt_rsp_val.find("+CSQ:") != std::string::npos ? std::stoi(opt_rsp_val.substr(opt_rsp_val.find(":") + 1)) : -1;

    return signal_quality;
}

template <typename AsyncReadOp>
bool LocalTransceiver::runWithTimeout(AsyncReadOp && op, boost::system::error_code & out_ec)
{
    boost::asio::deadline_timer timer(io_);
    bool                        timed_out = false;
    bool                        completed = false;

    timer.expires_from_now(boost::posix_time::milliseconds(SERIAL_TIMEOUT.count()));

    timer.async_wait([&](const boost::system::error_code & ec) {
        if (!ec && !completed) {
            timed_out = true;
            serial_.cancel();
        }
    });

    op([&](const boost::system::error_code & ec, std::size_t) {
        completed = true;
        out_ec    = ec;
        timer.cancel();
    });

    io_.run();
    io_.restart();

    if (timed_out) {
        out_ec = boost::asio::error::timed_out;
        return false;
    }

    return !out_ec;
}
