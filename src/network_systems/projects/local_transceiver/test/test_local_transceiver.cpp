/* IMPORTANT: Make sure only one instance of sailbot_workspace/scripts/run_virtual_iridium.sh is running */

#include <curl/curl.h>
#include <gtest/gtest.h>

#include <boost/process.hpp>
#include <boost/system/system_error.hpp>
#include <custom_interfaces/msg/detail/ais_ships__struct.hpp>
#include <custom_interfaces/msg/detail/helper_ais_ship__struct.hpp>
#include <custom_interfaces/msg/detail/helper_dimension__struct.hpp>
#include <custom_interfaces/msg/detail/helper_heading__struct.hpp>
#include <custom_interfaces/msg/detail/helper_lat_lon__struct.hpp>
#include <custom_interfaces/msg/detail/helper_rot__struct.hpp>
#include <custom_interfaces/msg/detail/helper_speed__struct.hpp>
#include <fstream>
#include <mutex>
#include <vector>

#include "at_cmds.h"
#include "cmn_hdrs/shared_constants.h"
#include "global_path.pb.h"
#include "local_transceiver.h"
#include "sensors.pb.h"

namespace bp = boost::process;

/* >>>>>README<<<<<<
Local Transceiver unit tests rely on two other programs: Virtual Iridium and HTTP Echo Server
1. Spawning a separate process for RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH doesn't work very well because the script
    spawns it's own subprocess for mock serial ports. You can spawn it ezpz, but cleaning up mock serial port
    subprocesses is a lot harder than you'd think. Hence, it's not done in the test code.
2. Virtual Iridium needs a valid HTTP POST endpoint for certain commands. RUN_HTTP_ECHO_SERVER_CMD runs a simple
   server that just echos whatever it receives.
   ***IMPORTANT***: Make sure the echo server is running on the host and port specified in the virtual iridium
                    --webhook_server_endpoint argument (default: 127.0.0.1:8081)
*/
class TestLocalTransceiver : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        bp::system("pkill -f http_echo_server");  // kill any currently running http_echo_server processes
        http_echo_server_proc_ = bp::child(RUN_HTTP_ECHO_SERVER_CMD, bp::std_out > stdout, bp::std_err > stderr);
        std::error_code e;
        if (!http_echo_server_proc_.running(e)) {
            throw std::runtime_error("Failed to start http echo server process! " + e.message());
        }
    }

    static void TearDownTestSuite() { http_echo_server_proc_.terminate(); }

    TestLocalTransceiver()
    {
        try {
            lcl_trns_ = new LocalTransceiver(LOCAL_TRANSCEIVER_TEST_PORT, SATELLITE_BAUD_RATE);
        } catch (boost::system::system_error & e) {
            std::stringstream ss;
            ss << "Failed to create Local Transceiver for tests, is only one instance of: \""
               << RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH << "\" running?" << std::endl;
            ss << e.what() << std::endl;
            throw std::runtime_error(ss.str());
        }
    }
    ~TestLocalTransceiver() override
    {
        lcl_trns_->stop();
        delete lcl_trns_;
    }

    LocalTransceiver * lcl_trns_;

private:
    static bp::child http_echo_server_proc_;
};
bp::child TestLocalTransceiver::http_echo_server_proc_ = {};

/**
 * @brief Verify debugSend
 */
TEST_F(TestLocalTransceiver, debugSendTest)
{
    auto opt_result = lcl_trns_->debugSend(AT::CHECK_CONN);
    EXPECT_TRUE(opt_result);
    std::string result = opt_result.value();
    EXPECT_TRUE(boost::algorithm::contains(result, AT::Line(AT::STATUS_OK).str_));
}

/**
 * @brief Send a binary string to virtual_iridium and verify it is received
 * Using gps, ais, wind, batteries, generic sensors, local path data
 */
TEST_F(TestLocalTransceiver, sendData)
{
    constexpr float   holder     = 14.3;
    constexpr int32_t holder_int = 11;

    // custom inferfaces used
    custom_interfaces::msg::GPS            gps;
    custom_interfaces::msg::AISShips       ais;
    custom_interfaces::msg::WindSensors    wind;
    custom_interfaces::msg::Batteries      batteries;
    custom_interfaces::msg::GenericSensors sensors;
    custom_interfaces::msg::LPathData      local_paths;

    // assign gps data
    gps.heading.set__heading(holder);
    gps.lat_lon.set__latitude(holder);
    gps.lat_lon.set__longitude(holder);
    gps.speed.set__speed(holder);

    // assign ais data
    custom_interfaces::msg::HelperAISShip   ship_one;
    custom_interfaces::msg::HelperHeading   heading_one;
    custom_interfaces::msg::HelperLatLon    lat_lon_one;
    custom_interfaces::msg::HelperSpeed     speed_one;
    custom_interfaces::msg::HelperROT       rotation_one;
    custom_interfaces::msg::HelperDimension width_one;
    custom_interfaces::msg::HelperDimension length_one;

    heading_one.set__heading(holder);
    lat_lon_one.set__latitude(holder);
    lat_lon_one.set__longitude(holder);
    speed_one.set__speed(holder);
    rotation_one.set__rot(holder_int);
    width_one.set__dimension(holder);
    length_one.set__dimension(holder);

    ship_one.set__id(holder_int);
    ship_one.set__cog(heading_one);
    ship_one.set__lat_lon(lat_lon_one);
    ship_one.set__sog(speed_one);
    ship_one.set__rot(rotation_one);
    ship_one.set__width(width_one);
    ship_one.set__length(length_one);

    ais.set__ships({ship_one});

    // assign wind data
    custom_interfaces::msg::WindSensor  wind_data_one;
    custom_interfaces::msg::WindSensor  wind_data_two;
    custom_interfaces::msg::HelperSpeed wind_speed;

    wind_data_one.set__direction(holder_int);
    wind_speed.set__speed(holder);
    wind_data_one.set__speed(wind_speed);
    wind_data_two.set__direction(holder_int);
    wind_data_two.set__speed(wind_speed);
    wind.set__wind_sensors({wind_data_one, wind_data_two});

    // assign batteries data
    custom_interfaces::msg::HelperBattery battery_one;
    custom_interfaces::msg::HelperBattery battery_two;

    battery_one.set__current(holder);
    battery_one.set__voltage(holder);
    battery_two.set__current(holder);
    battery_two.set__voltage(holder);
    batteries.set__batteries({battery_one, battery_two});

    // assign generic sensors data
    custom_interfaces::msg::HelperGenericSensor sensor;

    sensor.set__data(holder_int);
    sensor.set__id(holder_int);
    sensors.set__generic_sensors({sensor});

    // assign local path data
    custom_interfaces::msg::Path         local_path;
    custom_interfaces::msg::HelperLatLon lat_lon;
    lat_lon.set__latitude(holder);
    lat_lon.set__longitude(holder);
    local_path.set__waypoints({lat_lon});
    local_paths.set__local_path({local_path});

    // update sensors and send
    lcl_trns_->updateSensor(wind);
    lcl_trns_->updateSensor(gps);
    lcl_trns_->updateSensor(ais);
    lcl_trns_->updateSensor(batteries);
    lcl_trns_->updateSensor(sensors);
    lcl_trns_->updateSensor(local_paths);

    EXPECT_TRUE(lcl_trns_->send());
}

/**
 * @brief Verifies correct construction of status response object
 *        for at_cmds.h
 */
TEST_F(TestLocalTransceiver, ValidSBDRespose)
{
    std::string      response = "+SBDIX:0,1234,0,5678,9,2";
    AT::SBDStatusRsp status(response);

    EXPECT_EQ(status.MO_status_, 0);
    EXPECT_EQ(status.MOMSN_, 1234);
    EXPECT_EQ(status.MT_status_, 0);
    EXPECT_EQ(status.MTMSN_, 5678);
    EXPECT_EQ(status.MT_len_, 9);
    EXPECT_EQ(status.MT_queued_, 2);
}

/**
 * @brief Verifies exception is thrown for incorrect construction of status response object
 *        for at_cmds.h
 */
TEST_F(TestLocalTransceiver, InvalidSBDRespose)
{
    std::string responseNonInteger = "+SBDIX:hello,THIS,should,THROW,an,EXCEPTION";
    ASSERT_THROW(AT::SBDStatusRsp status(responseNonInteger), std::invalid_argument);
}

/**
 * @brief Verifies correct reporting of MO status (success, failure, no network)
 *        for at_cmds.h
 */
TEST_F(TestLocalTransceiver, MOStatusTest)
{
    std::string success    = "+SBDIX:0,1234,0,5678,9,2";
    std::string fail       = "+SBDIX:5,1234,0,5678,9,2";
    std::string no_network = "+SBDIX:32,1234,0,5678,9,2";

    AT::SBDStatusRsp success_response(success);
    AT::SBDStatusRsp failed_response(fail);
    AT::SBDStatusRsp no_network_service_response(no_network);

    EXPECT_TRUE(success_response.MOSuccess());
    EXPECT_FALSE(failed_response.MOSuccess());
    EXPECT_FALSE(no_network_service_response.MOSuccess());
}

/**
 * @brief Verifies that message from remote server is correctly parsed
 */
TEST_F(TestLocalTransceiver, parseInMsgValid)
{
    constexpr float                                   holder = 14.3;
    std::vector<custom_interfaces::msg::HelperLatLon> waypoints;

    // protobuf
    Polaris::GlobalPath path;

    Polaris::Waypoint * waypoint_a = path.add_waypoints();
    waypoint_a->set_latitude(holder);
    waypoint_a->set_longitude(holder);

    Polaris::Waypoint * waypoint_b = path.add_waypoints();
    waypoint_b->set_latitude(holder);
    waypoint_b->set_longitude(holder);

    // convert protobuf to string
    std::string serialized_test = path.SerializeAsString();

    custom_interfaces::msg::Path parsed_test = LocalTransceiver::parseInMsg(serialized_test);
    EXPECT_EQ(parsed_test.waypoints[0].latitude, holder);
    EXPECT_EQ(parsed_test.waypoints[0].longitude, holder);
    EXPECT_EQ(parsed_test.waypoints[1].latitude, holder);
    EXPECT_EQ(parsed_test.waypoints[1].longitude, holder);
}

std::mutex port_mutex;

TEST_F(TestLocalTransceiver, testMailboxBlackbox)
{
    std::lock_guard<std::mutex> lock(port_mutex);  // because same port is being used

    std::string holder  = "curl -X POST -F \"test=1234\" http://localhost:8080";
    std::string holder2 = "printf \"at+sbdix\r\" > $LOCAL_TRANSCEIVER_TEST_PORT";

    system(holder.c_str());   //NOLINT
    system(holder2.c_str());  //NOLINT

    std::optional<std::string> response = lcl_trns_->readRsp();
    std::cout << *response << std::endl;
}

TEST_F(TestLocalTransceiver, parseReceiveMessageBlackbox)
{
    std::lock_guard<std::mutex> lock(port_mutex);

    constexpr float     holder = 14.3;
    Polaris::GlobalPath sample_data;

    Polaris::Waypoint * waypoint_a = sample_data.add_waypoints();
    waypoint_a->set_latitude(holder);
    waypoint_a->set_longitude(holder);
    Polaris::Waypoint * waypoint_b = sample_data.add_waypoints();
    waypoint_b->set_latitude(holder);
    waypoint_b->set_longitude(holder);

    std::string serialized_data;
    ASSERT_TRUE(sample_data.SerializeToString(&serialized_data));

    std::ofstream outfile("/tmp/serialized_data.bin", std::ios::binary);
    outfile.write(serialized_data.data(), static_cast<std::streamsize>(serialized_data.size()));
    outfile.close();

    std::string holder2 = "curl -X POST -F \"data=@/tmp/serialized_data.bin\" http://localhost:8080";
    std::system(holder2.c_str());  //NOLINT

    custom_interfaces::msg::Path received_data = lcl_trns_->receive();

    Polaris::GlobalPath global_path;
    for (const auto & waypoint : received_data.waypoints) {
        Polaris::Waypoint * new_waypoint = global_path.add_waypoints();
        new_waypoint->set_latitude(waypoint.latitude);
        new_waypoint->set_longitude(waypoint.longitude);
    }

    if (global_path.waypoints_size() > 0) {
        ASSERT_EQ(global_path.waypoints_size(), sample_data.waypoints_size())
          << "Mismatch in number of waypoints received.";
        ASSERT_EQ(global_path.waypoints(0).latitude(), holder);
        ASSERT_EQ(global_path.waypoints(0).longitude(), holder);
    } else {
        std::cout << "No waypoints received." << std::endl;
    }
}
