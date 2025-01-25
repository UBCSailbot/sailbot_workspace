#include "local_transceiver.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>
#include <custom_interfaces/msg/ais_ships.hpp>
#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/generic_sensors.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <custom_interfaces/msg/l_path_data.hpp>
#include <custom_interfaces/msg/wind_sensors.hpp>
#include <exception>
#include <regex>
#include <stdexcept>
#include <string>

#include "at_cmds.h"
#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "global_path.pb.h"
#include "sensors.pb.h"
#include "waypoint.pb.h"

using boost::system::error_code;
using Polaris::Sensors;
namespace bio = boost::asio;

void LocalTransceiver::updateSensor(msg::GPS gps)
{
    sensors_.mutable_gps()->set_heading(gps.heading.heading);
    sensors_.mutable_gps()->set_latitude(gps.lat_lon.latitude);
    sensors_.mutable_gps()->set_longitude(gps.lat_lon.longitude);
    sensors_.mutable_gps()->set_speed(gps.speed.speed);
}

void LocalTransceiver::updateSensor(msg::AISShips ships)
{
    sensors_.clear_ais_ships();
    for (const msg::HelperAISShip & ship : ships.ships) {
        Sensors::Ais * new_ship = sensors_.add_ais_ships();
        new_ship->set_id(ship.id);
        new_ship->set_cog(ship.cog.heading);
        new_ship->set_latitude(ship.lat_lon.latitude);
        new_ship->set_longitude(ship.lat_lon.longitude);
        new_ship->set_sog(ship.sog.speed);
        new_ship->set_rot(ship.rot.rot);
        new_ship->set_width(ship.width.dimension);
        new_ship->set_length(ship.length.dimension);
    }
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

void LocalTransceiver::updateSensor(msg::GenericSensors msg)
{
    sensors_.clear_data_sensors();
    for (const msg::HelperGenericSensor & sensors_data : msg.generic_sensors) {
        Sensors::Generic * new_sensor = sensors_.add_data_sensors();
        new_sensor->set_data(sensors_data.data);
        new_sensor->set_id(sensors_data.id);
    }
}

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

LocalTransceiver::LocalTransceiver(const std::string & port_name, const uint32_t baud_rate) : serial_(io_, port_name)
{
    serial_.set_option(bio::serial_port_base::baud_rate(baud_rate));
    // Set a timeout for read/write operations on the serial port
    setsockopt(serial_.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &TIMEOUT, sizeof(TIMEOUT));
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
        std::cerr << "Failed to serialize sensors string" << std::endl;
        std::cerr << sensors.DebugString() << std::endl;
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

    std::string write_bin_cmd_str = AT::write_bin::CMD + std::to_string(data.size());  //according to specs
    AT::Line    at_write_cmd(write_bin_cmd_str);

    static constexpr int MAX_NUM_RETRIES = 20;  // allow retries because the connection is imperfect
    for (int i = 0; i < MAX_NUM_RETRIES; i++) {
        if (!send(at_write_cmd)) {
            continue;
        }

        if (!rcvRsps({
              at_write_cmd,
              AT::Line(AT::DELIMITER),
              AT::Line(AT::RSP_READY),
              AT::Line("\n"),
            })) {
            continue;
        }

        std::string msg_str = data + checksum(data);
        AT::Line    msg(msg_str);
        if (!send(msg)) {
            continue;
        }

        if (!rcvRsps({
              AT::Line(AT::DELIMITER),
              AT::Line(AT::write_bin::rsp::SUCCESS),
              AT::Line("\n"),
              AT::Line(AT::DELIMITER),
              AT::Line(AT::STATUS_OK),
              AT::Line("\n"),
            })) {
            continue;
        }

        // Check SBD Session status to see if data was sent successfully
        // NEEDS AN ACTIVE SERVER ON $WEBHOOK_SERVER_ENDPOINT OR VIRTUAL IRIDIUM WILL CRASH
        static const AT::Line sbdix_cmd = AT::Line(AT::SBD_SESSION);
        if (!send(sbdix_cmd)) {
            continue;
        }

        if (!rcvRsps({
              AT::Line("\r"),
              sbdix_cmd,
              AT::Line(AT::DELIMITER),
            })) {
            continue;
        }

        auto opt_rsp = readRsp();
        if (!opt_rsp) {
            continue;
        }

        // This string will look something like:
        // "+SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>\r\n\r\nOK\r"
        // on success
        // Don't bother to check for OK\r as MO status will tell us if it succeeded or not
        std::string              opt_rsp_val = opt_rsp.value();
        std::vector<std::string> sbd_status_vec;
        boost::algorithm::split(sbd_status_vec, opt_rsp_val, boost::is_any_of(AT::DELIMITER));

        AT::SBDStatusRsp rsp(sbd_status_vec[0]);
        if (rsp.MOSuccess()) {
            return true;
        }
    }
    std::cerr << "Failed to transmit data to satellite!" << std::endl;
    std::cerr << sensors.DebugString() << std::endl;
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

custom_interfaces::msg::Path LocalTransceiver::receive()
{
    static constexpr int MAX_NUM_RETRIES = 20;
    for (int i = 0; i <= MAX_NUM_RETRIES; i++) {
        if (i == MAX_NUM_RETRIES) {
            return parseInMsg("-1");
        }
        static const AT::Line check_conn_cmd = AT::Line(AT::CHECK_CONN);
        if (!send(check_conn_cmd)) {
            continue;
        }

        if (!rcvRsps({check_conn_cmd, AT::Line(AT::DELIMITER), AT::Line(AT::STATUS_OK), AT::Line("\n")})) {
            continue;
        }

        static const AT::Line disable_ctrlflow_cmd = AT::Line(AT::DSBL_CTRLFLOW);
        if (!send(disable_ctrlflow_cmd)) {
            continue;
        }

        if (!rcvRsps({disable_ctrlflow_cmd, AT::Line(AT::DELIMITER), AT::Line(AT::STATUS_OK), AT::Line("\n")})) {
            continue;
        }

        static const AT::Line sbdix_cmd = AT::Line(AT::SBD_SESSION);
        if (!send(sbdix_cmd)) {
            continue;
        }

        auto opt_rsp = readRsp();
        if (!opt_rsp) {
            continue;
        }

        std::string              opt_rsp_val = opt_rsp.value();
        std::vector<std::string> sbd_status_vec;
        boost::algorithm::split(sbd_status_vec, opt_rsp_val, boost::is_any_of(AT::DELIMITER));

        std::string sbdix_value;
        for (const auto & element : sbd_status_vec) {
            if (element.find("SBDIX:") != std::string::npos) {
                sbdix_value = element;
                break;
            }
        }

        AT::SBDStatusRsp rsp(sbdix_value);

        if (rsp.MO_status_ == 0) {
            if (rsp.MT_status_ == 0) {
                return parseInMsg("-1");
            } else if (rsp.MT_status_ == 1) {  //NOLINT
                break;
            } else if (rsp.MT_status_ == 2) {
                continue;
            }
        } else {
            continue;
        }
    }

    std::string receivedDataBuffer;
    for (int i = 0; i < MAX_NUM_RETRIES; i++) {
        static const AT::Line message_to_queue_cmd = AT::Line(AT::DNLD_TO_QUEUE);
        if (!send(message_to_queue_cmd)) {
            continue;
        }

        if (!rcvRsps({message_to_queue_cmd, AT::Line("\n")})) {
            continue;
        }

        auto buffer_data = readRsp();
        if (!buffer_data) {
            continue;
        }

        std::regex re(
          R"(name=\"data\"; filename=\"[^\"]*\"\r?\n(?:.*\r?\n)*\r?\n([\s\S]*?)\r?\n--)", std::regex::ECMAScript);

        std::smatch match;

        if (std::regex_search(*buffer_data, match, re)) {
            *buffer_data = match[1];
            std::stringstream ss;
            ss << *buffer_data;
            std::cout << ss.str() << std::endl;
        } else {
            std::cout << "No match found." << std::endl;
        }

        receivedDataBuffer = buffer_data.value();
        break;
    }

    custom_interfaces::msg::Path to_publish = parseInMsg(receivedDataBuffer);
    return to_publish;
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

bool LocalTransceiver::rcvRsp(const AT::Line & expected_rsp)
{
    bio::streambuf buf;
    error_code     ec;
    // Caution: will hang if another proccess is reading from the same serial port
    bio::read(serial_, buf, bio::transfer_exactly(expected_rsp.str_.size()), ec);
    if (ec) {
        std::cerr << "Failed to read with error: " << ec.message() << std::endl;
        return false;
    }
    std::string outstr = streambufToStr(buf);
    if (outstr != expected_rsp.str_) {
        std::cerr << "Expected to read: \"" << expected_rsp.str_ << "\"\nbut read: \"" << outstr << "\"" << std::endl;
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

std::optional<std::string> LocalTransceiver::readRsp()
{
    bio::streambuf buf;
    error_code     ec;

    // Caution: will hang if another proccess is reading from serial port
    bio::read_until(serial_, buf, AT::DELIMITER, ec);
    if (ec) {
        return std::nullopt;
    }

    std::string rsp_str = streambufToStr(buf);
    rsp_str.pop_back();  // Remove the "\n"
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
