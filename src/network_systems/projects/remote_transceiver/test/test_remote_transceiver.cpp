#include <curl/curl.h>
#include <gtest/gtest.h>

#include <array>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/beast/http/status.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <random>
#include <span>
#include <string>
#include <thread>

#include "cmn_hdrs/shared_constants.h"
#include "remote_transceiver.h"
#include "sailbot_db.h"
#include "sensors.pb.h"
#include "util_db.h"
#include "waypoint.pb.h"

using Polaris::GlobalPath;
using Polaris::Sensors;
using remote_transceiver::HTTPServer;
using remote_transceiver::Listener;
using remote_transceiver::TESTING_HOST;
using remote_transceiver::TESTING_PORT;
namespace http_client = remote_transceiver::http_client;

static const std::string  test_db_name = "test";
static std::random_device g_rd         = std::random_device();  // random number sampler
static uint32_t           g_rand_seed  = g_rd();                // seed used for random number generation
static std::mt19937       g_mt(g_rand_seed);                    // initialize random number generator with seed
static UtilDB             g_test_db(test_db_name, MONGODB_CONN_STR, std::make_shared<std::mt19937>(g_mt));

class TestRemoteTransceiver : public ::testing::Test
{
protected:
    static constexpr int NUM_THREADS = 4;
    // Need to wait after receiving an HTTP response from the server
    static constexpr auto WAIT_AFTER_RES = std::chrono::milliseconds(200);

    // Network objects that are shared amongst all HTTP test suites
    static bio::io_context          io_;
    static std::vector<std::thread> io_threads_;
    static SailbotDB                server_db_;
    static bio::ip::address         addr_;

    static void SetUpTestSuite()
    {
        std::make_shared<remote_transceiver::Listener>(
          TestRemoteTransceiver::io_, tcp::endpoint{TestRemoteTransceiver::addr_, TESTING_PORT},
          std::move(TestRemoteTransceiver::server_db_))
          ->run();

        for (std::thread & io_thread : io_threads_) {
            io_thread = std::thread([]() { io_.run(); });
        }
    }

    static void TearDownTestSuite()
    {
        io_.stop();
        for (std::thread & io_thread : io_threads_) {
            io_thread.join();
        }
    }

    TestRemoteTransceiver() { g_test_db.cleanDB(); }

    static size_t WriteCallback(void * contents, size_t size, size_t nmemb, void * userp)
    {
        (static_cast<std::string *>(userp))->append(static_cast<char *>(contents), size * nmemb);
        return size * nmemb;
    }

    ~TestRemoteTransceiver() override {}
};

// Initialize static objects
bio::io_context  TestRemoteTransceiver::io_{TestRemoteTransceiver::NUM_THREADS};
std::vector      TestRemoteTransceiver::io_threads_ = std::vector<std::thread>(NUM_THREADS);
SailbotDB        TestRemoteTransceiver::server_db_  = SailbotDB(test_db_name, MONGODB_CONN_STR);
bio::ip::address TestRemoteTransceiver::addr_       = bio::ip::make_address(TESTING_HOST);

/**
 * @brief Test HTTP GET request sending and handling. Currently just retrieves a placeholder string.
 *
 */
TEST_F(TestRemoteTransceiver, TestGet)
{
    auto [status, result] =
      http_client::get({TESTING_HOST, std::to_string(TESTING_PORT), remote_transceiver::targets::ROOT});
    EXPECT_EQ(status, http::status::ok);
    EXPECT_EQ(result, "PLACEHOLDER\r\n");
}

/**
 * @brief Create a formatted string that matches the body of POST requests from Iridium
 *        https://docs.rockblock.rock7.com/reference/receiving-mo-messages-via-http-webhook
 *
 * @param params Params structure
 * @return formatted request body
 */

std::string createPostBody(remote_transceiver::MOMsgParams::Params params);

std::string createSensorPostBody(remote_transceiver::MOMsgParams::Params params)

{
    std::ostringstream s;
    s << "imei=" << params.imei_ << "&serial=" << params.serial_ << "&momsn=" << params.momsn_
      << "&transmit_time=" << params.transmit_time_ << "&iridium_latitude=" << params.lat_
      << "&iridium_longitude=" << params.lon_ << "&iridium_cep=" << params.cep_ << "&data=" << params.data_;
    return s.str();
}

/**
 * @brief Test that we can POST sensor data to the server
 *
 */
TEST_F(TestRemoteTransceiver, TestPostSensors)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    auto [rand_sensors, rand_info] = g_test_db.genRandData(UtilDB::getTimestamp());

    std::string rand_sensors_str;
    ASSERT_TRUE(rand_sensors.SerializeToString(&rand_sensors_str));
    Polaris::Sensors test;
    test.ParseFromString(rand_sensors_str);
    // This query is comprised entirely of arbitrary values exccept for .data_
    std::string query = createSensorPostBody(
      {.imei_          = 0,
       .serial_        = 0,
       .momsn_         = 1,
       .transmit_time_ = rand_info.timestamp_,
       .lat_           = rand_info.lat_,
       .lon_           = rand_info.lon_,
       .cep_           = rand_info.cep_,
       .data_          = rand_sensors_str});
    http::status status = http_client::post(
      {TESTING_HOST, std::to_string(TESTING_PORT), remote_transceiver::targets::SENSORS},
      "application/x-www-form-urlencoded", query);

    EXPECT_EQ(status, http::status::ok);
    std::this_thread::sleep_for(WAIT_AFTER_RES);

    std::array<Sensors, 1>                expected_sensors = {rand_sensors};
    std::array<SailbotDB::RcvdMsgInfo, 1> expected_info    = {rand_info};
    EXPECT_TRUE(g_test_db.verifyDBWrite(expected_sensors, expected_info));
}

/**
 * @brief Test that the server can multiple POST sensor requests at once
 *
 */
TEST_F(TestRemoteTransceiver, TestPostSensorsMult)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure

    constexpr int                          NUM_REQS = 50;  // Keep this number under 60 to simplify timestamp logic
    std::array<std::string, NUM_REQS>      queries;
    std::array<std::thread, NUM_REQS>      req_threads;
    std::array<http::status, NUM_REQS>     res_statuses;
    std::array<Polaris::Sensors, NUM_REQS> expected_sensors;
    std::array<SailbotDB::RcvdMsgInfo, NUM_REQS> expected_info;

    std::tm tm = UtilDB::getTimestamp();
    // Prepare all queries
    for (int i = 0; i < NUM_REQS; i++) {
        // Timestamps are only granular to the second, so if we want to maintain document ordering by time
        // without adding a lot of 1 second delays, then the time must be modified
        tm.tm_sec                      = i;
        auto [rand_sensors, rand_info] = g_test_db.genRandData(tm);
        expected_sensors[i]            = rand_sensors;
        expected_info[i]               = rand_info;
        std::string rand_sensors_str;
        ASSERT_TRUE(rand_sensors.SerializeToString(&rand_sensors_str));
        Polaris::Sensors test;
        test.ParseFromString(rand_sensors_str);
        // This query is comprised entirely of arbitrary values exccept for .data_
        queries[i] = createSensorPostBody(
          {.imei_          = 0,
           .serial_        = 0,
           .momsn_         = 1,
           .transmit_time_ = rand_info.timestamp_,
           .lat_           = rand_info.lat_,
           .lon_           = rand_info.lon_,
           .cep_           = rand_info.cep_,
           .data_          = rand_sensors_str});
    }

    // Send all requests at once
    for (int i = 0; i < NUM_REQS; i++) {
        req_threads[i] = std::thread([&queries, &res_statuses, i]() {
            std::string query = queries[i];
            res_statuses[i]   = http_client::post(
                {TESTING_HOST, std::to_string(TESTING_PORT), remote_transceiver::targets::SENSORS},
                "application/x-www-form-urlencoded", query);
        });
    }

    // Wait for all requests to finish
    for (int i = 0; i < NUM_REQS; i++) {
        req_threads[i].join();
        EXPECT_EQ(res_statuses[i], http::status::ok);
    }
    std::this_thread::sleep_for(WAIT_AFTER_RES);

    // Check that DB is updated properly for all requests
    EXPECT_TRUE(g_test_db.verifyDBWrite(expected_sensors, expected_info));
}

TEST_F(TestRemoteTransceiver, rockblockWebServerExample)
{
    CURL *      curl;
    CURLcode    res;
    std::string readBuffer;

    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();

    if (curl != nullptr) {
        curl_easy_setopt(
          curl, CURLOPT_URL, "http://localhost:8100/?data=thisistestdata&ec=B&imei=300434065264590&username=myuser");

        curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");

        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);

        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        res = curl_easy_perform(curl);

        if (res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            // EXPECT_TRUE(false);
        } else {
            EXPECT_EQ("FAILED,11,No RockBLOCK with this IMEI found on your account", readBuffer);
        }

        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();
}

/**
 * @brief Test that we can POST global path data
 *
 */
TEST_F(TestRemoteTransceiver, TestPostGlobalPath)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    auto [rand_global_path, rand_global_path_timestamp] = g_test_db.genGlobalData(UtilDB::getTimestamp());

    std::string rand_global_path_str;
    ASSERT_TRUE(rand_global_path.SerializeToString(&rand_global_path_str));
    Polaris::GlobalPath test;
    test.ParseFromString(rand_global_path_str);

    boost::property_tree::ptree global_path_json;
    boost::property_tree::ptree waypoints_arr;

    for (const auto & waypoint : rand_global_path.waypoints()) {
        boost::property_tree::ptree waypoint_node;
        waypoint_node.put("latitude", waypoint.latitude());
        waypoint_node.put("longitude", waypoint.longitude());
        waypoints_arr.push_back(std::make_pair("", waypoint_node));
    }

    global_path_json.add_child("waypoints", waypoints_arr);
    global_path_json.put("timestamp", rand_global_path_timestamp);

    std::stringstream global_path_ss;
    boost::property_tree::json_parser::write_json(global_path_ss, global_path_json);

    http::status status = http_client::post(
      {TESTING_HOST, std::to_string(TESTING_PORT), remote_transceiver::targets::GLOBAL_PATH},
      "application/x-www-form-urlencoded", global_path_ss.str());  //change url as per global path specs

    EXPECT_EQ(status, http::status::ok);
    std::this_thread::sleep_for(WAIT_AFTER_RES);

    std::array<std::string, 1>         expected_response    = {"FAILED"};
    std::array<std::string, 1>         expected_error       = {"11"};
    std::array<std::string, 1>         expected_message     = {"No RockBLOCK with this IMEI found on your account"};
    std::array<std::string, 1>         expected_timestamp   = {rand_global_path_timestamp};
    std::array<Polaris::GlobalPath, 1> expected_global_path = {rand_global_path};

    EXPECT_TRUE(g_test_db.verifyDBWrite_GlobalPath(expected_global_path, expected_timestamp));
    EXPECT_TRUE(
      g_test_db.verifyDBWrite_IridiumResponse(expected_response, expected_error, expected_message, expected_timestamp));
}

TEST_F(TestRemoteTransceiver, TestPostGlobalPathMult)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure

    constexpr int                             NUM_REQS = 3;  // Keep this number under 60 to simplify timestamp logic
    std::array<std::string, NUM_REQS>         queries;
    std::array<std::thread, NUM_REQS>         req_threads;
    std::array<http::status, NUM_REQS>        res_statuses;
    std::array<Polaris::GlobalPath, NUM_REQS> expected_globalpaths;
    std::array<std::string, NUM_REQS>         expected_timestamps;
    std::array<std::string, NUM_REQS>         expected_response;
    std::array<std::string, NUM_REQS>         expected_error;
    std::array<std::string, NUM_REQS>         expected_message;

    std::tm tm = UtilDB::getTimestamp();
    //Prepare all queries
    for (int i = 0; i < NUM_REQS; i++) {
        // Timestamps are only granular to the second, so if we want to maintain document ordering by time
        // without adding a lot of 1 second delays, then the time must be modified
        tm.tm_sec                               = i;
        auto [rand_globalpaths, rand_timestamp] = g_test_db.genGlobalData(tm);
        expected_globalpaths[i]                 = rand_globalpaths;
        expected_timestamps[i]                  = rand_timestamp;
        expected_response[i]                    = "FAILED";
        expected_error[i]                       = "11";
        expected_message[i]                     = "No RockBLOCK with this IMEI found on your account";
        std::string rand_globalpath_str;
        ASSERT_TRUE(rand_globalpaths.SerializeToString(&rand_globalpath_str));
        Polaris::GlobalPath test;
        test.ParseFromString(rand_globalpath_str);

        boost::property_tree::ptree global_path_json;
        boost::property_tree::ptree waypoints_arr;

        for (const auto & waypoint : rand_globalpaths.waypoints()) {
            boost::property_tree::ptree waypoint_node;
            waypoint_node.put("latitude", waypoint.latitude());
            waypoint_node.put("longitude", waypoint.longitude());
            waypoints_arr.push_back(std::make_pair("", waypoint_node));
        }

        global_path_json.add_child("waypoints", waypoints_arr);
        global_path_json.put("timestamp", rand_timestamp);

        std::stringstream global_path_ss;
        boost::property_tree::json_parser::write_json(global_path_ss, global_path_json);

        queries[i] = global_path_ss.str();
    }

    for (int i = 0; i < NUM_REQS; i++) {
        req_threads[i] = std::thread([&queries, &res_statuses, i]() {
            std::string query = queries[i];
            res_statuses[i]   = http_client::post(
                {TESTING_HOST, std::to_string(TESTING_PORT), remote_transceiver::targets::GLOBAL_PATH},
                "application/x-www-form-urlencoded", query);
        });
    }

    // Wait for all requests to finish
    for (int i = 0; i < NUM_REQS; i++) {
        req_threads[i].join();
        EXPECT_EQ(res_statuses[i], http::status::ok);
    }
    std::this_thread::sleep_for(WAIT_AFTER_RES);

    EXPECT_TRUE(g_test_db.verifyDBWrite_GlobalPath(expected_globalpaths, expected_timestamps));
    EXPECT_TRUE(g_test_db.verifyDBWrite_IridiumResponse(
      expected_response, expected_error, expected_message, expected_timestamps));
}
