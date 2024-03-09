#include <gtest/gtest.h>

#include <array>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/beast/http/status.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/document/value.hpp>
#include <bsoncxx/document/view.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mongocxx/client.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/cursor.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/options/find.hpp>
#include <random>
#include <span>
#include <string>
#include <thread>

#include "cmn_hdrs/shared_constants.h"
#include "remote_transceiver.h"
#include "sailbot_db.h"
#include "sensors.pb.h"
#include "waypoint.pb.h"

constexpr int NUM_AIS_SHIPS       = 15;  // arbitrary number
constexpr int NUM_GENERIC_SENSORS = 5;   // arbitrary number
constexpr int NUM_PATH_WAYPOINTS  = 5;   // arbitrary number

using Polaris::Sensors;
using remote_transceiver::HTTPServer;
using remote_transceiver::Listener;
using remote_transceiver::TESTING_HOST;
using remote_transceiver::TESTING_PORT;
namespace http_client = remote_transceiver::http_client;

//Child class of SailbotDB that includes additional database utility functions to help testing
class TestDB : public SailbotDB
{
public:
    static constexpr auto TEST_DB = "test";
    TestDB() : SailbotDB(TEST_DB, MONGODB_CONN_STR) {}

    /**
     * @brief Delete all documents in all collections
     */
    void cleanDB()
    {
        mongocxx::pool::entry entry = pool_->acquire();
        mongocxx::database    db    = (*entry)[db_name_];

        mongocxx::collection gps_coll        = db[COLLECTION_GPS];
        mongocxx::collection ais_coll        = db[COLLECTION_AIS_SHIPS];
        mongocxx::collection generic_coll    = db[COLLECTION_DATA_SENSORS];
        mongocxx::collection batteries_coll  = db[COLLECTION_BATTERIES];
        mongocxx::collection wind_coll       = db[COLLECTION_WIND_SENSORS];
        mongocxx::collection local_path_coll = db[COLLECTION_LOCAL_PATH];

        gps_coll.delete_many(bsoncxx::builder::basic::make_document());
        ais_coll.delete_many(bsoncxx::builder::basic::make_document());
        generic_coll.delete_many(bsoncxx::builder::basic::make_document());
        batteries_coll.delete_many(bsoncxx::builder::basic::make_document());
        wind_coll.delete_many(bsoncxx::builder::basic::make_document());
        local_path_coll.delete_many(bsoncxx::builder::basic::make_document());
    }

    /**
     * @brief Retrieve all sensors from the database sorted by timestamp
     *
     * @param num_docs expected number of documents for each collection, default 1
     *
     * @return Vector of sensors objects: gps, ais, generic, batteries, wind, local path
     * @return Vector of timestamps
     *         both vectors will be num_docs in size
     */
    std::pair<std::vector<Sensors>, std::vector<std::string>> dumpSensors(size_t num_docs = 1)
    {
        std::vector<Sensors>     sensors_vec(num_docs);
        std::vector<std::string> timestamp_vec(num_docs);
        mongocxx::pool::entry    entry = pool_->acquire();
        mongocxx::database       db    = (*entry)[db_name_];
        // Set the find options to sort by timestamp
        bsoncxx::document::value order = bsoncxx::builder::stream::document{} << "timestamp" << 1
                                                                              << bsoncxx::builder::stream::finalize;
        mongocxx::options::find opts = mongocxx::options::find{};
        opts.sort(order.view());

        // gps
        mongocxx::collection gps_coll = db[COLLECTION_GPS];
        mongocxx::cursor     gps_docs = gps_coll.find({}, opts);
        EXPECT_EQ(gps_coll.count_documents({}), num_docs)
          << "Error: TestDB should only have " << num_docs << " documents per collection";

        for (auto [i, gps_docs_it] = std::tuple{size_t{0}, gps_docs.begin()}; i < num_docs; i++, gps_docs_it++) {
            Sensors &                     sensors   = sensors_vec[i];
            std::string &                 timestamp = timestamp_vec[i];
            const bsoncxx::document::view gps_doc   = *gps_docs_it;

            Sensors::Gps * gps = sensors.mutable_gps();
            gps->set_latitude(static_cast<float>(gps_doc["latitude"].get_double().value));
            gps->set_longitude(static_cast<float>(gps_doc["longitude"].get_double().value));
            gps->set_speed(static_cast<float>(gps_doc["speed"].get_double().value));
            gps->set_heading(static_cast<float>(gps_doc["heading"].get_double().value));
            timestamp = gps_doc["timestamp"].get_utf8().value.to_string();
        }

        // ais ships
        mongocxx::collection ais_coll = db[COLLECTION_AIS_SHIPS];
        mongocxx::cursor     ais_docs = ais_coll.find({}, opts);
        EXPECT_EQ(ais_coll.count_documents({}), num_docs)
          << "Error: TestDB should only have " << num_docs << " documents per collection";

        for (auto [i, ais_docs_it] = std::tuple{size_t{0}, ais_docs.begin()}; i < num_docs; i++, ais_docs_it++) {
            Sensors &                     sensors       = sensors_vec[i];
            const std::string &           timestamp     = timestamp_vec[i];
            const bsoncxx::document::view ais_ships_doc = *ais_docs_it;

            for (bsoncxx::array::element ais_ships_doc : ais_ships_doc["ships"].get_array().value) {
                Sensors::Ais * ais_ship = sensors.add_ais_ships();
                ais_ship->set_id(static_cast<uint32_t>(ais_ships_doc["id"].get_int64().value));
                ais_ship->set_latitude(static_cast<float>(ais_ships_doc["latitude"].get_double().value));
                ais_ship->set_longitude(static_cast<float>(ais_ships_doc["longitude"].get_double().value));
                ais_ship->set_sog(static_cast<float>(ais_ships_doc["sog"].get_double().value));
                ais_ship->set_cog(static_cast<float>(ais_ships_doc["cog"].get_double().value));
                ais_ship->set_rot(static_cast<float>(ais_ships_doc["rot"].get_double().value));
                ais_ship->set_width(static_cast<float>(ais_ships_doc["width"].get_double().value));
                ais_ship->set_length(static_cast<float>(ais_ships_doc["length"].get_double().value));
            }
            EXPECT_EQ(sensors.ais_ships().size(), NUM_AIS_SHIPS) << "Size mismatch when reading AIS ships from DB";
            EXPECT_EQ(ais_ships_doc["timestamp"].get_utf8().value.to_string(), timestamp)
              << "Document timestamp mismatch";
        }

        // generic sensor
        mongocxx::collection generic_coll        = db[COLLECTION_DATA_SENSORS];
        mongocxx::cursor     generic_sensor_docs = generic_coll.find({}, opts);
        EXPECT_EQ(generic_coll.count_documents({}), num_docs)
          << "Error: TestDB should only have " << num_docs << " documents per collection";

        for (auto [i, generic_sensor_docs_it] = std::tuple{size_t{0}, generic_sensor_docs.begin()}; i < num_docs;
             i++, generic_sensor_docs_it++) {
            Sensors &                     sensors     = sensors_vec[i];
            const std::string &           timestamp   = timestamp_vec[i];
            const bsoncxx::document::view generic_doc = *generic_sensor_docs_it;

            for (bsoncxx::array::element generic_doc : generic_doc["genericSensors"].get_array().value) {
                Sensors::Generic * generic = sensors.add_data_sensors();
                generic->set_id(static_cast<uint32_t>(generic_doc["id"].get_int64().value));
                generic->set_data(static_cast<uint64_t>(generic_doc["data"].get_int64().value));
            }
            EXPECT_EQ(generic_doc["timestamp"].get_utf8().value.to_string(), timestamp)
              << "Document timestamp mismatch";
        }

        // battery
        mongocxx::collection batteries_coll      = db[COLLECTION_BATTERIES];
        mongocxx::cursor     batteries_data_docs = batteries_coll.find({}, opts);
        EXPECT_EQ(batteries_coll.count_documents({}), num_docs)
          << "Error: TestDB should only have " << num_docs << " documents per collection";

        for (auto [i, batteries_doc_it] = std::tuple{size_t{0}, batteries_data_docs.begin()}; i < num_docs;
             i++, batteries_doc_it++) {
            Sensors &                     sensors       = sensors_vec[i];
            const std::string &           timestamp     = timestamp_vec[i];
            const bsoncxx::document::view batteries_doc = *batteries_doc_it;

            for (bsoncxx::array::element batteries_doc : batteries_doc["batteries"].get_array().value) {
                Sensors::Battery * battery = sensors.add_batteries();
                battery->set_voltage(static_cast<float>(batteries_doc["voltage"].get_double().value));
                battery->set_current(static_cast<float>(batteries_doc["current"].get_double().value));
            }
            EXPECT_EQ(sensors.batteries().size(), NUM_BATTERIES) << "Size mismatch when reading batteries from DB";
            EXPECT_EQ(batteries_doc["timestamp"].get_utf8().value.to_string(), timestamp)
              << "Document timestamp mismatch";
        }

        // wind sensor
        mongocxx::collection wind_coll         = db[COLLECTION_WIND_SENSORS];
        mongocxx::cursor     wind_sensors_docs = wind_coll.find({}, opts);
        EXPECT_EQ(wind_coll.count_documents({}), num_docs)
          << "Error: TestDB should only have " << num_docs << " documents per collection";

        for (auto [i, wind_doc_it] = std::tuple{size_t{0}, wind_sensors_docs.begin()}; i < num_docs;
             i++, wind_doc_it++) {
            Sensors &                     sensors   = sensors_vec[i];
            const std::string &           timestamp = timestamp_vec[i];
            const bsoncxx::document::view wind_doc  = *wind_doc_it;
            for (bsoncxx::array::element wind_doc : wind_doc["windSensors"].get_array().value) {
                Sensors::Wind * wind = sensors.add_wind_sensors();
                wind->set_speed(static_cast<float>(wind_doc["speed"].get_double().value));
                wind->set_direction(static_cast<int16_t>(wind_doc["direction"].get_int32().value));
            }
            EXPECT_EQ(sensors.wind_sensors().size(), NUM_WIND_SENSORS)
              << "Size mismatch when reading batteries from DB";
            EXPECT_EQ(wind_doc["timestamp"].get_utf8().value.to_string(), timestamp) << "Document timestamp mismatch";
        }

        // local path
        mongocxx::collection path_coll       = db[COLLECTION_LOCAL_PATH];
        mongocxx::cursor     local_path_docs = path_coll.find({}, opts);
        EXPECT_EQ(path_coll.count_documents({}), num_docs)
          << "Error: TestDB should only have " << num_docs << " documents per collection";

        for (auto [i, path_doc_it] = std::tuple{size_t{0}, local_path_docs.begin()}; i < num_docs; i++, path_doc_it++) {
            Sensors &                     sensors   = sensors_vec[i];
            const std::string &           timestamp = timestamp_vec[i];
            const bsoncxx::document::view path_doc  = *path_doc_it;
            for (bsoncxx::array::element path_doc : path_doc["waypoints"].get_array().value) {
                Polaris::Waypoint * path = sensors.mutable_local_path_data()->add_waypoints();
                path->set_latitude(static_cast<float>(path_doc["latitude"].get_double().value));
                path->set_longitude(static_cast<float>(path_doc["longitude"].get_double().value));
            }
            EXPECT_EQ(sensors.local_path_data().waypoints_size(), NUM_PATH_WAYPOINTS)
              << "Size mismatch when reading path waypoints from DB";
            EXPECT_EQ(path_doc["timestamp"].get_utf8().value.to_string(), timestamp) << "Document timestamp mismatch";
        }

        return {sensors_vec, timestamp_vec};
    }
};

static TestDB             g_test_db   = TestDB();              // initialize the TestDB instance
static std::random_device g_rd        = std::random_device();  // random number sampler
static uint32_t           g_rand_seed = g_rd();                // seed used for random number generation
static std::mt19937       g_mt(g_rand_seed);                   // initialize random number generator with seed
// Use a static counter for creating testing timestamps as the internals of the
// remote transceiver do not care about the format.
// The counter is of type char because of how strings are sorted in alphabetical order,
// where a sequence of strings "1", "2", "10" are sorted to "1", "10", "2".
// Incrementing a char prevents this issue for up to 256 numbers as it increments
// '0', '1', ..., '9', '<LF>', ...
static char g_doc_num = '0';

class TestSailbotDB : public ::testing::Test
{
protected:
    TestSailbotDB()
    {
        g_test_db.cleanDB();
        g_doc_num = '0';
    }
    ~TestSailbotDB() override {}
};

/**
 * @brief generate random GPS data
 *
 * @param gps_data pointer to generated gps_data
 */
void * genRandGpsData(Sensors::Gps * gps_data)
{
    std::uniform_real_distribution<float> lat_dist(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> lon_dist(LON_LBND, LON_UBND);
    std::uniform_real_distribution<float> speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float> heading_dist(HEADING_LBND, HEADING_UBND);
    gps_data->set_latitude(lat_dist(g_mt));
    gps_data->set_longitude(lon_dist(g_mt));
    gps_data->set_speed(speed_dist(g_mt));
    gps_data->set_heading(heading_dist(g_mt));

    return gps_data;
}

/**
 * @brief generate random ais ships data
 *
 * @param ais_ship pointer to generated ais data
 */
void genRandAisData(Sensors::Ais * ais_ship)
{
    std::uniform_int_distribution<uint32_t> id_dist(0, UINT32_MAX);
    std::uniform_real_distribution<float>   lat_dist(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float>   lon_dist(LON_LBND, LON_UBND);
    std::uniform_real_distribution<float>   speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float>   heading_dist(HEADING_LBND, HEADING_UBND);
    std::uniform_real_distribution<float>   rot_dist(ROT_LBND, ROT_UBND);
    std::uniform_real_distribution<float>   width_dist(SHIP_DIMENSION_LBND, SHIP_DIMENSION_UBND);
    std::uniform_real_distribution<float>   length_dist(SHIP_DIMENSION_LBND, SHIP_DIMENSION_UBND);

    ais_ship->set_id(id_dist(g_mt));
    ais_ship->set_latitude(lat_dist(g_mt));
    ais_ship->set_longitude(lon_dist(g_mt));
    ais_ship->set_sog(speed_dist(g_mt));
    ais_ship->set_cog(heading_dist(g_mt));
    ais_ship->set_rot(rot_dist(g_mt));
    ais_ship->set_width(width_dist(g_mt));
    ais_ship->set_length(length_dist(g_mt));
}

/**
 * @brief generate random generic sensor data
 *
 * @return pointer to generated generic sensor data
 */
void genRandGenericSensorData(Sensors::Generic * generic_sensor)
{
    std::uniform_int_distribution<uint8_t>  id_generic(0, UINT8_MAX);
    std::uniform_int_distribution<uint64_t> data_generic(0, UINT64_MAX);

    generic_sensor->set_id(id_generic(g_mt));
    generic_sensor->set_data(data_generic(g_mt));
}

/**
 * @brief generate random battery data
 *
 * @return pointer to generated battery data
 */
void genRandBatteriesData(Sensors::Battery * battery)
{
    std::uniform_real_distribution<float> voltage_battery(BATT_VOLT_LBND, BATT_VOLT_UBND);
    std::uniform_real_distribution<float> current_battery(BATT_CURR_LBND, BATT_CURR_UBND);

    battery->set_voltage(voltage_battery(g_mt));
    battery->set_current(current_battery(g_mt));
}

/**
 * @brief generate random wind sensors data
 *
 * @return pointer to generated wind sensors data
 */
void genRandWindData(Sensors::Wind * wind_data)
{
    std::uniform_real_distribution<float> speed_wind(SPEED_LBND, SPEED_UBND);
    std::uniform_int_distribution<int>    direction_wind(WIND_DIRECTION_LBND, WIND_DIRECTION_UBND);

    wind_data->set_speed(speed_wind(g_mt));
    wind_data->set_direction(direction_wind(g_mt));
}

/**
 * @brief generate random path data
 *
 * @return pointer to generated path data
 */
void genRandPathData(Sensors::Path * path_data)
{
    std::uniform_real_distribution<float> latitude_path(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> longitude_path(LON_LBND, LON_UBND);

    for (int i = 0; i < NUM_PATH_WAYPOINTS; i++) {
        Polaris::Waypoint * waypoint = path_data->add_waypoints();
        waypoint->set_latitude(latitude_path(g_mt));
        waypoint->set_longitude(longitude_path(g_mt));
    }
}

/**
 * @brief Generate random data for all sensors
 *
 * @return Sensors object
 */
Sensors genRandSensors()
{
    Sensors sensors;

    // gps
    genRandGpsData(sensors.mutable_gps());

    // ais ships, TODO(): Polaris should be included as one of the AIS ships
    for (int i = 0; i < NUM_AIS_SHIPS; i++) {
        genRandAisData(sensors.add_ais_ships());
    }

    // generic sensors
    for (int i = 0; i < NUM_GENERIC_SENSORS; i++) {
        genRandGenericSensorData(sensors.add_data_sensors());
    }

    // batteries
    for (int i = 0; i < NUM_BATTERIES; i++) {
        genRandBatteriesData(sensors.add_batteries());
    }

    // wind sensors
    for (int i = 0; i < NUM_WIND_SENSORS; i++) {
        genRandWindData(sensors.add_wind_sensors());
    }

    // path waypoints
    genRandPathData(sensors.mutable_local_path_data());

    return sensors;
}

/**
 * @brief Generate random sensors and Iridium msg info
 *
 * @return std::pair<Sensors, SailbotDB::RcvdMsgInfo>
 */
std::pair<Sensors, SailbotDB::RcvdMsgInfo> genRandData()
{
    Sensors                rand_sensors = genRandSensors();
    SailbotDB::RcvdMsgInfo rand_info{
      .lat_       = 0,                             // Not processed yet, so just set to 0
      .lon_       = 0,                             // Not processed yet, so just set to 0
      .cep_       = 0,                             // Not processed yet, so just set to 0
      .timestamp_ = std::to_string(g_doc_num++)};  // increment counter after converting and storing
    return {rand_sensors, rand_info};
}

/**
 * @brief Query the database and check that the sensor and message are correct
 *
 * @param expected_sensors
 * @param expected_msg_info
 */
void verifyDBWrite(std::span<Sensors> expected_sensors, std::span<SailbotDB::RcvdMsgInfo> expected_msg_info)
{
    ASSERT_EQ(expected_sensors.size(), expected_msg_info.size()) << "Must have msg info for each set of Sensors";
    size_t num_docs                          = expected_sensors.size();
    auto [dumped_sensors, dumped_timestamps] = g_test_db.dumpSensors(num_docs);

    EXPECT_EQ(dumped_sensors.size(), num_docs);
    EXPECT_EQ(dumped_timestamps.size(), num_docs);

    for (size_t i = 0; i < num_docs; i++) {
        EXPECT_EQ(dumped_timestamps[i], expected_msg_info[i].timestamp_);

        // gps
        EXPECT_FLOAT_EQ(dumped_sensors[i].gps().latitude(), expected_sensors[i].gps().latitude());
        EXPECT_FLOAT_EQ(dumped_sensors[i].gps().longitude(), expected_sensors[i].gps().longitude());
        EXPECT_FLOAT_EQ(dumped_sensors[i].gps().speed(), expected_sensors[i].gps().speed());
        EXPECT_FLOAT_EQ(dumped_sensors[i].gps().heading(), expected_sensors[i].gps().heading());

        // ais ships
        for (int j = 0; j < NUM_AIS_SHIPS; j++) {
            const Sensors::Ais & dumped_ais_ship   = dumped_sensors[i].ais_ships(j);
            const Sensors::Ais & expected_ais_ship = expected_sensors[i].ais_ships(j);
            EXPECT_EQ(dumped_ais_ship.id(), expected_ais_ship.id());
            EXPECT_FLOAT_EQ(dumped_ais_ship.latitude(), expected_ais_ship.latitude());
            EXPECT_FLOAT_EQ(dumped_ais_ship.longitude(), expected_ais_ship.longitude());
            EXPECT_FLOAT_EQ(dumped_ais_ship.sog(), expected_ais_ship.sog());
            EXPECT_FLOAT_EQ(dumped_ais_ship.cog(), expected_ais_ship.cog());
            EXPECT_FLOAT_EQ(dumped_ais_ship.rot(), expected_ais_ship.rot());
            EXPECT_FLOAT_EQ(dumped_ais_ship.width(), expected_ais_ship.width());
            EXPECT_FLOAT_EQ(dumped_ais_ship.length(), expected_ais_ship.length());
        }

        // generic sensors
        for (int j = 0; j < NUM_GENERIC_SENSORS; j++) {
            const Sensors::Generic & dumped_data_sensor   = dumped_sensors[i].data_sensors(j);
            const Sensors::Generic & expected_data_sensor = expected_sensors[i].data_sensors(j);
            EXPECT_EQ(dumped_data_sensor.id(), expected_data_sensor.id());
            EXPECT_EQ(dumped_data_sensor.data(), expected_data_sensor.data());
        }

        // batteries
        for (int j = 0; j < NUM_BATTERIES; j++) {
            const Sensors::Battery & dumped_battery   = dumped_sensors[i].batteries(j);
            const Sensors::Battery & expected_battery = expected_sensors[i].batteries(j);
            EXPECT_EQ(dumped_battery.voltage(), expected_battery.voltage());
            EXPECT_EQ(dumped_battery.current(), expected_battery.current());
        }

        // wind sensors
        for (int j = 0; j < NUM_WIND_SENSORS; j++) {
            const Sensors::Wind & dumped_wind_sensor   = dumped_sensors[i].wind_sensors(j);
            const Sensors::Wind & expected_wind_sensor = expected_sensors[i].wind_sensors(j);
            EXPECT_EQ(dumped_wind_sensor.speed(), expected_wind_sensor.speed());
            EXPECT_EQ(dumped_wind_sensor.direction(), expected_wind_sensor.direction());
        }

        // path waypoints
        for (int j = 0; j < NUM_PATH_WAYPOINTS; j++) {
            const Polaris::Waypoint & dumped_path_waypoint   = dumped_sensors[i].local_path_data().waypoints(j);
            const Polaris::Waypoint & expected_path_waypoint = expected_sensors[i].local_path_data().waypoints(j);
            EXPECT_EQ(dumped_path_waypoint.latitude(), expected_path_waypoint.latitude());
            EXPECT_EQ(dumped_path_waypoint.longitude(), expected_path_waypoint.longitude());
        }
    }
}

/**
 * @brief Check that MongoDB is running
 */
TEST_F(TestSailbotDB, TestConnection)
{
    ASSERT_TRUE(g_test_db.testConnection()) << "MongoDB not running - remember to connect!";
}

/**
 * @brief Write random sensor data to the TestDB - read and verify said data
 */
TEST_F(TestSailbotDB, TestStoreSensors)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    auto [rand_sensors, rand_info] = genRandData();
    ASSERT_TRUE(g_test_db.storeNewSensors(rand_sensors, rand_info));

    std::array<Sensors, 1>                expected_sensors = {rand_sensors};
    std::array<SailbotDB::RcvdMsgInfo, 1> expected_info    = {rand_info};

    verifyDBWrite(expected_sensors, expected_info);
}

class TestHTTP : public TestSailbotDB
{
protected:
    static constexpr int NUM_THREADS = 4;
    // Need to wait after receiving an HTTP response from the server
    static constexpr auto WAIT_AFTER_RES = std::chrono::milliseconds(20);

    // Network objects that are shared amongst all HTTP test suites
    static bio::io_context          io_;
    static std::vector<std::thread> io_threads_;
    static SailbotDB                server_db_;
    static bio::ip::address         addr_;

    static void SetUpTestSuite()
    {
        std::make_shared<remote_transceiver::Listener>(
          TestHTTP::io_, tcp::endpoint{TestHTTP::addr_, TESTING_PORT}, std::move(TestHTTP::server_db_))
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

    TestHTTP()
    {
        // Automatically calls TestSailbotDB's constructor to setup tests
    }

    ~TestHTTP() override {}
};

// Initialize static objects
bio::io_context  TestHTTP::io_{TestHTTP::NUM_THREADS};
std::vector      TestHTTP::io_threads_ = std::vector<std::thread>(NUM_THREADS);
SailbotDB        TestHTTP::server_db_  = TestDB();
bio::ip::address TestHTTP::addr_       = bio::ip::make_address(TESTING_HOST);

/**
 * @brief Test HTTP GET request sending and handling. Currently just retrieves a placeholder string.
 *
 */
TEST_F(TestHTTP, TestGet)
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
std::string createPostBody(remote_transceiver::MOMsgParams::Params params)
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
TEST_F(TestHTTP, TestPostSensors)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    auto [rand_sensors, rand_info] = genRandData();

    std::string rand_sensors_str;
    ASSERT_TRUE(rand_sensors.SerializeToString(&rand_sensors_str));
    Polaris::Sensors test;
    test.ParseFromString(rand_sensors_str);
    // This query is comprised entirely of arbitrary values exccept for .data_
    std::string query = createPostBody(
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
    verifyDBWrite(expected_sensors, expected_info);
}

/**
 * @brief Test that the server can multiple POST requests at once
 *
 */
TEST_F(TestHTTP, TestPostSensorsMult)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure

    constexpr int                                NUM_REQS = 50;
    std::array<std::string, NUM_REQS>            queries;
    std::array<std::thread, NUM_REQS>            req_threads;
    std::array<http::status, NUM_REQS>           res_statuses;
    std::array<Polaris::Sensors, NUM_REQS>       expected_sensors;
    std::array<SailbotDB::RcvdMsgInfo, NUM_REQS> expected_info;

    // Prepare all queries
    for (int i = 0; i < NUM_REQS; i++) {
        auto [rand_sensors, rand_info] = genRandData();
        expected_sensors[i]            = rand_sensors;
        expected_info[i]               = rand_info;
        std::string rand_sensors_str;
        ASSERT_TRUE(rand_sensors.SerializeToString(&rand_sensors_str));
        Polaris::Sensors test;
        test.ParseFromString(rand_sensors_str);
        // This query is comprised entirely of arbitrary values exccept for .data_
        queries[i] = createPostBody(
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
    verifyDBWrite(expected_sensors, expected_info);
}
