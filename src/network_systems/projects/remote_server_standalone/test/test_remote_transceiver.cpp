#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/beast/http/status.hpp>
#include <chrono>
#include <cstdint>
#include <random>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "remote_transceiver.h"
#include "sailbot_db.h"
#include "sensors.pb.h"
#include "test_db_helper.h"

using Polaris::Sensors;
using remote_transceiver::TESTING_HOST;
using remote_transceiver::TESTING_PORT;
namespace http_client = remote_transceiver::http_client;

static const std::string  test_db_name = "test";
static std::random_device g_rd         = std::random_device();  // random number sampler
static uint32_t           g_rand_seed  = g_rd();                // seed used for random number generation
static std::mt19937       g_mt(g_rand_seed);                    // initialize random number generator with seed
static UtilDB             g_test_db(test_db_name, SailbotDB::MONGODB_CONN_STR(), std::make_shared<std::mt19937>(g_mt));

class TestRemoteTransceiver : public ::testing::Test
{
protected:
    static constexpr int NUM_THREADS = 4;
    // Need to wait after receiving an HTTP response from the server
    static constexpr auto WAIT_AFTER_RES = std::chrono::milliseconds(1000);

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

    ~TestRemoteTransceiver() override {}
};

// Initialize static objects
bio::io_context  TestRemoteTransceiver::io_{TestRemoteTransceiver::NUM_THREADS};
std::vector<std::thread> TestRemoteTransceiver::io_threads_ = std::vector<std::thread>(NUM_THREADS);
SailbotDB        TestRemoteTransceiver::server_db_  = SailbotDB(test_db_name, SailbotDB::MONGODB_CONN_STR());
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

std::string bytesToHex(const std::string &bytes)
{
    static const char hex_digits[] = "0123456789abcdef";
    std::ostringstream s;
    for (unsigned char c : bytes)
    {
        s << hex_digits[c >> 4] << hex_digits[c & 0x0f];
    }
    return s.str();
}

std::string createSensorPostBody(remote_transceiver::MOMsgParams::Params params)
{
    std::ostringstream s;
    s << "imei=" << params.imei_ << "&serial=" << params.serial_ << "&momsn=" << params.momsn_
      << "&transmit_time=" << params.transmit_time_ << "&iridium_latitude=" << params.lat_
      << "&iridium_longitude=" << params.lon_ << "&iridium_cep=" << params.cep_ << "&data="
      << bytesToHex(params.data_);
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

    constexpr int                                NUM_REQS = 5;  // Keep this number under 60 to simplify timestamp logic
    std::array<std::string, NUM_REQS>            queries;
    std::array<std::thread, NUM_REQS>            req_threads;
    std::array<http::status, NUM_REQS>           res_statuses;
    std::array<Polaris::Sensors, NUM_REQS>       expected_sensors;
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

/**
 * @brief Test that the old global path POST route is no longer handled by remote transceiver.
 *
 */
TEST_F(TestRemoteTransceiver, TestPostGlobalPathNotFound)
{
    http::status status = http_client::post(
      {TESTING_HOST, std::to_string(TESTING_PORT), "/global-path"}, "application/json", "{}");

    EXPECT_EQ(status, http::status::bad_request);
}

/**
 * @brief POST one set of random sensors to the server
 *
 * @return The sensors and message info that were sent
 */
std::pair<Sensors, SailbotDB::RcvdMsgInfo> postRandSensors()
{
    auto [rand_sensors, rand_info] = g_test_db.genRandData(UtilDB::getTimestamp());
    std::string rand_sensors_str;
    EXPECT_TRUE(rand_sensors.SerializeToString(&rand_sensors_str));
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
    return {rand_sensors, rand_info};
}

/**
 * @brief Test that a new voyage is named <date>_<n> and becomes the only active voyage
 *
 */
TEST_F(TestRemoteTransceiver, TestStartNewVoyage)
{
    EXPECT_EQ(g_test_db.getActiveVoyage(), "");

    const std::string first = g_test_db.startNewVoyage();
    EXPECT_TRUE(std::regex_match(first, std::regex("^[0-9]{4}-[0-9]{2}-[0-9]{2}_1$"))) << first;
    EXPECT_EQ(g_test_db.getActiveVoyage(), first);

    // Same day, so the suffix counts up instead of reusing the name
    const std::string second = g_test_db.startNewVoyage();
    EXPECT_EQ(second, first.substr(0, first.size() - 1) + "2");
    EXPECT_EQ(g_test_db.getActiveVoyage(), second);
}

/**
 * @brief Test that switching the active voyage never leaves a moment with no active voyage, even while several
 *        activations run at once
 *
 */
TEST_F(TestRemoteTransceiver, TestSwitchActiveVoyageConcurrently)
{
    constexpr int     NUM_WRITERS     = 4;
    constexpr int     NUM_ACTIVATIONS = 25;
    const std::string first           = g_test_db.startNewVoyage();
    const std::string second          = g_test_db.startNewVoyage();

    std::atomic<bool> done{false};
    std::atomic<int>  reads_without_voyage{0};
    std::thread       reader([&]() {
        while (!done) {
            if (g_test_db.getActiveVoyage().empty()) {
                reads_without_voyage++;
            }
        }
    });

    std::array<std::thread, NUM_WRITERS> writers;
    for (int i = 0; i < NUM_WRITERS; i++) {
        writers[i] = std::thread([&, i]() {
            for (int n = 0; n < NUM_ACTIVATIONS; n++) {
                g_test_db.activateVoyage(i % 2 == 0 ? first : second);
            }
        });
    }
    for (std::thread & writer : writers) {
        writer.join();
    }
    done = true;
    reader.join();

    EXPECT_EQ(reads_without_voyage, 0);
    const std::string active = g_test_db.getActiveVoyage();
    EXPECT_TRUE(active == first || active == second) << active;
}

/**
 * @brief Test that POSTed sensors are tagged with the active voyage
 *
 */
TEST_F(TestRemoteTransceiver, TestPostSensorsTaggedWithVoyage)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    const std::string voyage       = g_test_db.startNewVoyage();
    auto [rand_sensors, rand_info] = postRandSensors();
    std::this_thread::sleep_for(WAIT_AFTER_RES);

    std::array<Sensors, 1>                expected_sensors = {rand_sensors};
    std::array<SailbotDB::RcvdMsgInfo, 1> expected_info    = {rand_info};
    EXPECT_TRUE(g_test_db.verifyDBWrite(expected_sensors, expected_info));
    EXPECT_TRUE(g_test_db.verifyVoyageTags(voyage));
}

/**
 * @brief Test that POSTed sensors are stored without a voyage field when no voyage is active
 *
 */
TEST_F(TestRemoteTransceiver, TestPostSensorsUntagged)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    postRandSensors();
    std::this_thread::sleep_for(WAIT_AFTER_RES);

    EXPECT_TRUE(g_test_db.verifyVoyageTags(""));
}

/**
 * @brief Test that stored timestamps always use the percent-encoded form RockBLOCK sends
 *
 */
TEST(TestSailbotDB, TestNormalizeTimestamp)
{
    const std::string encoded = "26-08-21%2012%3A30%3A05";
    EXPECT_EQ(SailbotDB::normalizeTimestamp(encoded), encoded);
    EXPECT_EQ(SailbotDB::normalizeTimestamp("26-08-21 12:30:05"), encoded);
    EXPECT_EQ(SailbotDB::normalizeTimestamp("26-08-21+12:30:05"), encoded);
    EXPECT_EQ(SailbotDB::normalizeTimestamp("26-08-21%2012%3a30%3a05"), encoded);
    EXPECT_EQ(SailbotDB::normalizeTimestamp("not a time"), "not a time");
}

/**
 * @brief Test that the password is escaped in the Atlas connection string, so characters like @ : / % cannot break it
 *
 */
TEST(TestSailbotDB, TestAtlasConnStrEscapesPassword)
{
    EXPECT_EQ(
      SailbotDB::atlasConnStr("pa@ss:w/rd%1"),
      "mongodb+srv://software:pa%40ss%3Aw%2Frd%251@dev.khxge.mongodb.net/?appName=dev");
    EXPECT_EQ(
      SailbotDB::atlasConnStr("Plain-pass_1.~"),
      "mongodb+srv://software:Plain-pass_1.~@dev.khxge.mongodb.net/?appName=dev");
}

/**
 * @brief Test that a database error is reported as a failed store. An exception would escape the server's detached
 *        POST thread and terminate the whole server.
 *
 */
TEST(TestSailbotDB, TestStoreNewSensorsDbError)
{
    SailbotDB unreachable(test_db_name, "mongodb://127.0.0.1:1/?serverSelectionTimeoutMS=200");
    auto [sensors, info] = g_test_db.genRandData(UtilDB::getTimestamp());
    EXPECT_FALSE(unreachable.storeNewSensors(sensors, info));
}
