#include "local_transceiver.h"

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
#include <stdexcept>
#include <string>

#include "at_cmds.h"
#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "sensors.pb.h"
#include "waypoint.pb.h"

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
    for (const msg::HelperLatLon & local_data : localData.local_path.waypoints) {
        Sensors::Path *     new_local = sensors_.mutable_local_path_data();
        Polaris::Waypoint * waypoint  = new_local->add_waypoints();
        waypoint->set_latitude(local_data.latitude);
        waypoint->set_longitude(local_data.longitude);
    }
}

Sensors LocalTransceiver::sensors() { return sensors_; }

LocalTransceiver::LocalTransceiver(const std::string & port_name, const uint32_t baud_rate) : serial_(io_, port_name)
{
    serial_.set_option(bio::serial_port_base::baud_rate(baud_rate));
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

    static constexpr int MAX_NUM_RETRIES = 20;
    for (int i = 0; i < MAX_NUM_RETRIES; i++) {
        std::string sbdwbCommand = "AT+SBDWB=" + std::to_string(data.size()) + "\r";
        send(sbdwbCommand + data + "\r");

        std::string checksumCommand = std::to_string(data.size()) + checksum(data) + "\r";
        send(data + "+" + checksumCommand + "\r");

        // Check SBD Session status to see if data was sent successfully
        send(AT::SBD_SESSION);
        std::string rsp_str = readLine();
        readLine();  // empty line after response
        if (checkOK()) {
            try {
                AT::SBDStatusResponse rsp(rsp_str);
                if (rsp.MOSuccess()) {
                    return true;
                }
            } catch (std::invalid_argument & e) {
                /* Catch response parsing exceptions */
            }
        }
    }
    return false;
}

std::string LocalTransceiver::debugSend(const std::string & cmd)
{
    send(cmd);

    std::string response = readLine();  // Read and capture the response
    readLine();                         // Check if there is an empty line after respones
    return response;
}

std::string LocalTransceiver::receive()
{
    std::string receivedData = readLine();
    return receivedData;
}

void LocalTransceiver::send(const std::string & cmd) { bio::write(serial_, bio::buffer(cmd, cmd.size())); }

std::string LocalTransceiver::parseInMsg(const std::string & msg)
{
    //TODO(jng468): implement function
    (void)msg;
    return "placeholder";
}

std::string LocalTransceiver::readLine()
{
    bio::streambuf buf;

    // Caution: will hang if another proccess is reading from serial port
    bio::read_until(serial_, buf, AT::DELIMITER);
    return std::string(
      bio::buffers_begin(buf.data()), bio::buffers_begin(buf.data()) + static_cast<int64_t>(buf.data().size()));
}

bool LocalTransceiver::checkOK()
{
    std::string status = readLine();
    return status == AT::STATUS_OK;
}

std::string LocalTransceiver::checksum(const std::string & data)
{
    uint16_t counter = 0;
    for (char c : data) {
        counter += static_cast<uint8_t>(c);
    }

    std::stringstream ss;
    ss << std::hex << std::setw(4) << std::setfill('0') << counter;
    return ss.str();
}
