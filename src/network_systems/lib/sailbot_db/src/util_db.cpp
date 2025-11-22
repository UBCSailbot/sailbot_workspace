#include "util_db.h"

#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/document/value.hpp>
#include <bsoncxx/document/view.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/cursor.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/options/find.hpp>
#include <random>

#include "cmn_hdrs/shared_constants.h"
#include "sailbot_db/inc/sailbot_db.h"
#include "sailbot_db/inc/util_db.h"
#include "utils/utils.h"

using Polaris::GlobalPath;
using Polaris::Sensors;

void UtilDB::cleanDB()
{
    mongocxx::pool::entry entry = pool_->acquire();
    mongocxx::database    db    = (*entry)[db_name_];

    mongocxx::collection gps_coll              = db[COLLECTION_GPS];
    mongocxx::collection ais_coll              = db[COLLECTION_AIS_SHIPS];
    mongocxx::collection generic_coll          = db[COLLECTION_DATA_SENSORS];
    mongocxx::collection batteries_coll        = db[COLLECTION_BATTERIES];
    mongocxx::collection wind_coll             = db[COLLECTION_WIND_SENSORS];
    mongocxx::collection local_path_coll       = db[COLLECTION_LOCAL_PATH];
    mongocxx::collection global_path_coll      = db[COLLECTION_GLOBAL_PATH];
    mongocxx::collection iridium_response_coll = db[COLLECTION_IRIDIUM_RESPONSE];

    gps_coll.delete_many(bsoncxx::builder::basic::make_document());
    ais_coll.delete_many(bsoncxx::builder::basic::make_document());
    generic_coll.delete_many(bsoncxx::builder::basic::make_document());
    batteries_coll.delete_many(bsoncxx::builder::basic::make_document());
    wind_coll.delete_many(bsoncxx::builder::basic::make_document());
    local_path_coll.delete_many(bsoncxx::builder::basic::make_document());
    global_path_coll.delete_many(bsoncxx::builder::basic::make_document());
    iridium_response_coll.delete_many(bsoncxx::builder::basic::make_document());
}

Sensors UtilDB::genRandSensors()
{
    Sensors sensors;

    // gps
    genRandGpsData(*sensors.mutable_gps());

    // ais ships, TODO(): Polaris should be included as one of the AIS ships
    for (int i = 0; i < NUM_AIS_SHIPS; i++) {
        genRandAisData(*sensors.add_ais_ships());
    }

    // generic sensors
    for (int i = 0; i < NUM_GENERIC_SENSORS; i++) {
        genRandGenericSensorData(*sensors.add_data_sensors());
    }

    // batteries
    for (int i = 0; i < NUM_BATTERIES; i++) {
        genRandBatteryData(*sensors.add_batteries());
    }

    // wind sensors
    for (int i = 0; i < NUM_WIND_SENSORS; i++) {
        genRandWindData(*sensors.add_wind_sensors());
    }

    // path waypoints
    genRandPathData(*sensors.mutable_local_path_data());

    return sensors;
}

GlobalPath UtilDB::genGlobalPath()
{
    GlobalPath global_path;
    genGlobalPathData(global_path);
    return global_path;
}

std::tm UtilDB::getTimestamp()
{
    // Get the current time
    std::time_t t  = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm *   tm = std::gmtime(&t);  // NOLINT(concurrency-mt-unsafe)
    // tm stores years since 1900 by default, the schema expects years since 2000
    tm->tm_year -= 100;  // NOLINT(readability-magic-numbers)
    return *tm;
}

std::pair<Sensors, SailbotDB::RcvdMsgInfo> UtilDB::genRandData(const std::tm & tm)
{
    Sensors rand_sensors = genRandSensors();

    SailbotDB::RcvdMsgInfo rand_info{
      .lat_       = 0,  // Not processed yet, so just set to 0
      .lon_       = 0,  // Not processed yet, so just set to 0
      .cep_       = 0,  // Not processed yet, so just set to 0
      .timestamp_ = SailbotDB::mkTimestamp(tm)};
    return {rand_sensors, rand_info};
}

std::pair<Polaris::GlobalPath, std::string> UtilDB::genGlobalData(const std::tm & tm)
{
    Polaris::GlobalPath global_path_data = genGlobalPath();

    std::string global_timestamp = {SailbotDB::mkTimestamp(tm)};
    return {global_path_data, global_timestamp};
}

bool UtilDB::verifyDBWrite(std::span<Sensors> expected_sensors, std::span<SailbotDB::RcvdMsgInfo> expected_msg_info)
{
    utils::FailTracker tracker;

    auto expectEQ = [&tracker]<not_float T>(T rcvd, T expected, const std::string & err_msg) -> void {
        tracker.track(utils::checkEQ(rcvd, expected, err_msg));
    };
    auto expectFloatEQ = [&tracker]<std::floating_point T>(T rcvd, T expected, const std::string & err_msg) -> void {
        tracker.track(utils::checkEQ(rcvd, expected, err_msg));
    };

    expectEQ(expected_sensors.size(), expected_msg_info.size(), "Must have msg info for each set of Sensors");
    size_t num_docs                          = expected_sensors.size();
    auto [dumped_sensors, dumped_timestamps] = dumpSensors(tracker, num_docs);

    expectEQ(dumped_sensors.size(), num_docs, "");
    expectEQ(dumped_timestamps.size(), num_docs, "");

    for (size_t i = 0; i < num_docs; i++) {
        expectEQ(dumped_timestamps[i], expected_msg_info[i].timestamp_, "");

        // gps
        expectFloatEQ(dumped_sensors[i].gps().latitude(), expected_sensors[i].gps().latitude(), "");
        expectFloatEQ(dumped_sensors[i].gps().longitude(), expected_sensors[i].gps().longitude(), "");
        expectFloatEQ(dumped_sensors[i].gps().speed(), expected_sensors[i].gps().speed(), "");
        expectFloatEQ(dumped_sensors[i].gps().heading(), expected_sensors[i].gps().heading(), "");

        // ais ships
        for (int j = 0; j < NUM_AIS_SHIPS; j++) {
            const Sensors::Ais & dumped_ais_ship   = dumped_sensors[i].ais_ships(j);
            const Sensors::Ais & expected_ais_ship = expected_sensors[i].ais_ships(j);
            expectEQ(dumped_ais_ship.id(), expected_ais_ship.id(), "");
            expectFloatEQ(dumped_ais_ship.latitude(), expected_ais_ship.latitude(), "");
            expectFloatEQ(dumped_ais_ship.longitude(), expected_ais_ship.longitude(), "");
            expectFloatEQ(dumped_ais_ship.sog(), expected_ais_ship.sog(), "");
            expectFloatEQ(dumped_ais_ship.cog(), expected_ais_ship.cog(), "");
            expectFloatEQ(dumped_ais_ship.rot(), expected_ais_ship.rot(), "");
            expectFloatEQ(dumped_ais_ship.width(), expected_ais_ship.width(), "");
            expectFloatEQ(dumped_ais_ship.length(), expected_ais_ship.length(), "");
        }

        // generic sensors
        for (int j = 0; j < NUM_GENERIC_SENSORS; j++) {
            const Sensors::Generic & dumped_data_sensor   = dumped_sensors[i].data_sensors(j);
            const Sensors::Generic & expected_data_sensor = expected_sensors[i].data_sensors(j);
            expectEQ(dumped_data_sensor.id(), expected_data_sensor.id(), "");
            expectEQ(dumped_data_sensor.data(), expected_data_sensor.data(), "");
        }

        // batteries
        for (int j = 0; j < NUM_BATTERIES; j++) {
            const Sensors::Battery & dumped_battery   = dumped_sensors[i].batteries(j);
            const Sensors::Battery & expected_battery = expected_sensors[i].batteries(j);
            expectFloatEQ(dumped_battery.voltage(), expected_battery.voltage(), "");
            expectFloatEQ(dumped_battery.current(), expected_battery.current(), "");
        }

        // wind sensors
        for (int j = 0; j < NUM_WIND_SENSORS; j++) {
            const Sensors::Wind & dumped_wind_sensor   = dumped_sensors[i].wind_sensors(j);
            const Sensors::Wind & expected_wind_sensor = expected_sensors[i].wind_sensors(j);
            expectFloatEQ(dumped_wind_sensor.speed(), expected_wind_sensor.speed(), "");
            expectEQ(dumped_wind_sensor.direction(), expected_wind_sensor.direction(), "");
        }

        // path waypoints
        for (int j = 0; j < NUM_PATH_WAYPOINTS; j++) {
            const Polaris::Waypoint & dumped_path_waypoint   = dumped_sensors[i].local_path_data().waypoints(j);
            const Polaris::Waypoint & expected_path_waypoint = expected_sensors[i].local_path_data().waypoints(j);
            expectFloatEQ(dumped_path_waypoint.latitude(), expected_path_waypoint.latitude(), "");
            expectFloatEQ(dumped_path_waypoint.longitude(), expected_path_waypoint.longitude(), "");
        }
    }
    return !tracker.failed();
}

bool UtilDB::verifyDBWrite_GlobalPath(
  std::span<GlobalPath> expected_globalpath, std::span<std::string> expected_timestamp)
{
    utils::FailTracker tracker;

    auto expectEQ = [&tracker]<not_float T>(T rcvd, T expected, const std::string & err_msg) -> void {
        tracker.track(utils::checkEQ(rcvd, expected, err_msg));
    };
    auto expectFloatEQ = [&tracker]<std::floating_point T>(T rcvd, T expected, const std::string & err_msg) -> void {
        tracker.track(utils::checkEQ(rcvd, expected, err_msg));
    };

    expectEQ(expected_globalpath.size(), expected_timestamp.size(), "Must have a timestamp for global path");
    size_t num_docs                             = expected_globalpath.size();
    auto [dumped_globalpath, dumped_timestamps] = dumpGlobalpath(tracker, num_docs);

    expectEQ(dumped_globalpath.size(), num_docs, "");
    expectEQ(dumped_timestamps.size(), num_docs, "");

    for (size_t i = 0; i < num_docs; i++) {
        expectEQ(dumped_timestamps[i], expected_timestamp[i], "");

        // path waypoints
        for (int j = 0; j < NUM_PATH_WAYPOINTS; j++) {
            const Polaris::Waypoint & dumped_path_waypoint   = dumped_globalpath[i].waypoints(j);
            const Polaris::Waypoint & expected_path_waypoint = expected_globalpath[i].waypoints(j);
            expectFloatEQ(dumped_path_waypoint.latitude(), expected_path_waypoint.latitude(), "");
            expectFloatEQ(dumped_path_waypoint.longitude(), expected_path_waypoint.longitude(), "");
        }
    }
    return !tracker.failed();
}

bool UtilDB::verifyDBWrite_IridiumResponse(
  std::span<std::string> expected_response, std::span<std::string> expected_error,
  std::span<std::string> expected_message, std::span<std::string> expected_timestamp)
{
    utils::FailTracker tracker;

    auto expectEQ = [&tracker]<not_float T>(T rcvd, T expected, const std::string & err_msg) -> void {
        tracker.track(utils::checkEQ(rcvd, expected, err_msg));
    };

    expectEQ(expected_response.size(), expected_timestamp.size(), "Must have a timestamp for each response");
    size_t num_docs                                                         = expected_response.size();
    auto [dumped_response, dumped_error, dumped_message, dumped_timestamps] = dumpIridiumResponse(tracker, num_docs);

    expectEQ(dumped_response.size(), num_docs, "");
    expectEQ(dumped_error.size(), num_docs, "");
    expectEQ(dumped_message.size(), num_docs, "");
    expectEQ(dumped_timestamps.size(), num_docs, "");

    for (size_t i = 0; i < num_docs; i++) {
        expectEQ(dumped_response[i], expected_response[i], "");
        expectEQ(dumped_error[i], expected_error[i], "");
        expectEQ(dumped_message[i], expected_message[i], "");
        expectEQ(dumped_timestamps[i], expected_timestamp[i], "");
    }
    return !tracker.failed();
}

std::tuple<std::vector<std::string>, std::vector<std::string>, std::vector<std::string>, std::vector<std::string>>
UtilDB::dumpIridiumResponse(utils::FailTracker & tracker, size_t num_docs)
{
    auto expectEQ = [&tracker]<not_float T>(T rcvd, T expected, const std::string & err_msg) -> void {
        tracker.track(utils::checkEQ(rcvd, expected, err_msg));
    };

    std::vector<std::string> response_vec(num_docs);
    std::vector<std::string> error_vec(num_docs);
    std::vector<std::string> message_vec(num_docs);
    std::vector<std::string> timestamp_vec(num_docs);
    mongocxx::pool::entry    entry = pool_->acquire();
    mongocxx::database       db    = (*entry)[db_name_];

    // Set the find options to sort by timestamp, don't need?
    bsoncxx::document::value order = bsoncxx::builder::stream::document{} << "timestamp" << 1
                                                                          << bsoncxx::builder::stream::finalize;
    mongocxx::options::find opts = mongocxx::options::find{};
    opts.sort(order.view());

    // iridium response
    mongocxx::collection path_coll             = db[COLLECTION_IRIDIUM_RESPONSE];
    mongocxx::cursor     iridium_response_docs = path_coll.find({}, opts);

    expectEQ(
      static_cast<uint64_t>(path_coll.count_documents({})), num_docs,
      "Error: TestDB should only have " + std::to_string(num_docs) + " documents per collection");

    for (auto [i, path_doc_it] = std::tuple{size_t{0}, iridium_response_docs.begin()}; i < num_docs;
         i++, path_doc_it++) {
        std::string &                 response  = response_vec[i];
        std::string &                 error     = error_vec[i];
        std::string &                 message   = message_vec[i];
        std::string &                 timestamp = timestamp_vec[i];
        const bsoncxx::document::view path_doc  = *path_doc_it;
        response_vec[i]                         = path_doc["response"].get_utf8().value.to_string();
        error_vec[i]                            = path_doc["error"].get_utf8().value.to_string();
        message_vec[i]                          = path_doc["message"].get_utf8().value.to_string();
        timestamp_vec[i]                        = path_doc["timestamp"].get_utf8().value.to_string();

        expectEQ(path_doc["response"].get_utf8().value.to_string(), response, "Document response mismatch");
        expectEQ(path_doc["error"].get_utf8().value.to_string(), error, "Document error mismatch");
        expectEQ(path_doc["message"].get_utf8().value.to_string(), message, "Document message mismatch");
        expectEQ(path_doc["timestamp"].get_utf8().value.to_string(), timestamp, "Document timestamp mismatch");
    }

    return {response_vec, error_vec, message_vec, timestamp_vec};
}

std::pair<std::vector<GlobalPath>, std::vector<std::string>> UtilDB::dumpGlobalpath(
  utils::FailTracker & tracker, size_t num_docs)
{
    auto expectEQ = [&tracker]<not_float T>(T rcvd, T expected, const std::string & err_msg) -> void {
        tracker.track(utils::checkEQ(rcvd, expected, err_msg));
    };

    std::vector<GlobalPath>  globalpath_vec(num_docs);
    std::vector<std::string> timestamp_vec(num_docs);
    mongocxx::pool::entry    entry = pool_->acquire();
    mongocxx::database       db    = (*entry)[db_name_];

    // Set the find options to sort by timestamp
    bsoncxx::document::value order = bsoncxx::builder::stream::document{} << "timestamp" << 1
                                                                          << bsoncxx::builder::stream::finalize;
    mongocxx::options::find opts = mongocxx::options::find{};
    opts.sort(order.view());

    // global path
    mongocxx::collection path_coll        = db[COLLECTION_GLOBAL_PATH];
    mongocxx::cursor     global_path_docs = path_coll.find({}, opts);
    expectEQ(
      static_cast<uint64_t>(path_coll.count_documents({})), num_docs,
      "Error: TestDB should only have " + std::to_string(num_docs) + " documents per collection");

    for (auto [i, path_doc_it] = std::tuple{size_t{0}, global_path_docs.begin()}; i < num_docs; i++, path_doc_it++) {
        GlobalPath &                  globalpath = globalpath_vec[i];
        const bsoncxx::document::view path_doc   = *path_doc_it;
        timestamp_vec[i]                         = path_doc["timestamp"].get_utf8().value.to_string();
        for (bsoncxx::array::element path_doc : path_doc["waypoints"].get_array().value) {
            Polaris::Waypoint * path = globalpath.add_waypoints();
            path->set_latitude(static_cast<float>(path_doc["latitude"].get_double().value));
            path->set_longitude(static_cast<float>(path_doc["longitude"].get_double().value));
        }
        expectEQ(globalpath.waypoints_size(), NUM_PATH_WAYPOINTS, "Size mismatch when reading path waypoints from DB");
        // expectEQ(path_doc["timestamp"].get_utf8().value.to_string(), timestamp, "Document timestamp mismatch");  // issue here
    }

    return {globalpath_vec, timestamp_vec};
}

std::pair<std::vector<Sensors>, std::vector<std::string>> UtilDB::dumpSensors(
  utils::FailTracker & tracker, size_t num_docs)
{
    auto expectEQ = [&tracker]<not_float T>(T rcvd, T expected, const std::string & err_msg) -> void {
        tracker.track(utils::checkEQ(rcvd, expected, err_msg));
    };

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
    expectEQ(
      static_cast<uint64_t>(gps_coll.count_documents({})), num_docs,
      "Error: TestDB should only have " + std::to_string(num_docs) + " documents per collection");

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
    expectEQ(
      static_cast<uint64_t>(ais_coll.count_documents({})), num_docs,
      "Error: TestDB should only have " + std::to_string(num_docs) + " documents per collection");

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
        expectEQ(sensors.ais_ships().size(), NUM_AIS_SHIPS, "Size mismatch when reading AIS ships from DB");
        expectEQ(ais_ships_doc["timestamp"].get_utf8().value.to_string(), timestamp, "Document timestamp mismatch");
    }

    // generic sensor
    mongocxx::collection generic_coll        = db[COLLECTION_DATA_SENSORS];
    mongocxx::cursor     generic_sensor_docs = generic_coll.find({}, opts);
    expectEQ(
      static_cast<uint64_t>(generic_coll.count_documents({})), num_docs,
      "Error: TestDB should only have " + std::to_string(num_docs) + " documents per collection");

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
        expectEQ(generic_doc["timestamp"].get_utf8().value.to_string(), timestamp, "Document timestamp mismatch");
    }

    // battery
    mongocxx::collection batteries_coll      = db[COLLECTION_BATTERIES];
    mongocxx::cursor     batteries_data_docs = batteries_coll.find({}, opts);
    expectEQ(
      static_cast<uint64_t>(batteries_coll.count_documents({})), num_docs,
      "Error: TestDB should only have " + std::to_string(num_docs) + " documents per collection");

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
        expectEQ(sensors.batteries().size(), NUM_BATTERIES, "Size mismatch when reading batteries from DB");
        expectEQ(batteries_doc["timestamp"].get_utf8().value.to_string(), timestamp, "Document timestamp mismatch");
    }

    // wind sensor
    mongocxx::collection wind_coll         = db[COLLECTION_WIND_SENSORS];
    mongocxx::cursor     wind_sensors_docs = wind_coll.find({}, opts);
    expectEQ(
      static_cast<uint64_t>(wind_coll.count_documents({})), num_docs,
      "Error: TestDB should only have " + std::to_string(num_docs) + " documents per collection");

    for (auto [i, wind_doc_it] = std::tuple{size_t{0}, wind_sensors_docs.begin()}; i < num_docs; i++, wind_doc_it++) {
        Sensors &                     sensors   = sensors_vec[i];
        const std::string &           timestamp = timestamp_vec[i];
        const bsoncxx::document::view wind_doc  = *wind_doc_it;
        for (bsoncxx::array::element wind_doc : wind_doc["windSensors"].get_array().value) {
            Sensors::Wind * wind = sensors.add_wind_sensors();
            wind->set_speed(static_cast<float>(wind_doc["speed"].get_double().value));
            wind->set_direction(static_cast<int16_t>(wind_doc["direction"].get_int32().value));
        }
        expectEQ(sensors.wind_sensors().size(), NUM_WIND_SENSORS, "Size mismatch when reading batteries from DB");
        expectEQ(wind_doc["timestamp"].get_utf8().value.to_string(), timestamp, "Document timestamp mismatch");
    }

    // local path
    mongocxx::collection path_coll       = db[COLLECTION_LOCAL_PATH];
    mongocxx::cursor     local_path_docs = path_coll.find({}, opts);
    expectEQ(
      static_cast<uint64_t>(path_coll.count_documents({})), num_docs,
      "Error: TestDB should only have " + std::to_string(num_docs) + " documents per collection");

    for (auto [i, path_doc_it] = std::tuple{size_t{0}, local_path_docs.begin()}; i < num_docs; i++, path_doc_it++) {
        Sensors &                     sensors   = sensors_vec[i];
        const std::string &           timestamp = timestamp_vec[i];
        const bsoncxx::document::view path_doc  = *path_doc_it;
        for (bsoncxx::array::element path_doc : path_doc["waypoints"].get_array().value) {
            Polaris::Waypoint * path = sensors.mutable_local_path_data()->add_waypoints();
            path->set_latitude(static_cast<float>(path_doc["latitude"].get_double().value));
            path->set_longitude(static_cast<float>(path_doc["longitude"].get_double().value));
        }
        expectEQ(
          sensors.local_path_data().waypoints_size(), NUM_PATH_WAYPOINTS,
          "Size mismatch when reading path waypoints from DB");
        expectEQ(path_doc["timestamp"].get_utf8().value.to_string(), timestamp, "Document timestamp mismatch");
    }

    return {sensors_vec, timestamp_vec};
}

UtilDB::UtilDB(const std::string & db_name, const std::string & mongodb_conn_str, std::shared_ptr<std::mt19937> rng)
: SailbotDB(db_name, mongodb_conn_str), rng_(rng)
{
}

void UtilDB::genRandGpsData(Sensors::Gps & gps_data)
{
    std::uniform_real_distribution<float> lat_dist(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> lon_dist(LON_LBND, LON_UBND);
    std::uniform_real_distribution<float> speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float> heading_dist(HEADING_LBND, HEADING_UBND);
    gps_data.set_latitude(lat_dist(*rng_));
    gps_data.set_longitude(lon_dist(*rng_));
    gps_data.set_speed(speed_dist(*rng_));
    gps_data.set_heading(heading_dist(*rng_));
}

void UtilDB::genRandAisData(Sensors::Ais & ais_ship)
{
    std::uniform_int_distribution<uint32_t> id_dist(0, UINT32_MAX);
    std::uniform_real_distribution<float>   lat_dist(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float>   lon_dist(LON_LBND, LON_UBND);
    std::uniform_real_distribution<float>   speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float>   heading_dist(HEADING_LBND, HEADING_UBND);
    std::uniform_real_distribution<float>   rot_dist(ROT_LBND, ROT_UBND);
    std::uniform_real_distribution<float>   width_dist(SHIP_DIMENSION_LBND, SHIP_DIMENSION_UBND);
    std::uniform_real_distribution<float>   length_dist(SHIP_DIMENSION_LBND, SHIP_DIMENSION_UBND);

    ais_ship.set_id(id_dist(*rng_));
    ais_ship.set_latitude(lat_dist(*rng_));
    ais_ship.set_longitude(lon_dist(*rng_));
    ais_ship.set_sog(speed_dist(*rng_));
    ais_ship.set_cog(heading_dist(*rng_));
    ais_ship.set_rot(rot_dist(*rng_));
    ais_ship.set_width(width_dist(*rng_));
    ais_ship.set_length(length_dist(*rng_));
}

void UtilDB::genRandGenericSensorData(Sensors::Generic & generic_sensor)
{
    std::uniform_int_distribution<uint8_t>  id_generic(0, UINT8_MAX);
    std::uniform_int_distribution<uint64_t> data_generic(0, UINT64_MAX);

    generic_sensor.set_id(id_generic(*rng_));
    generic_sensor.set_data(data_generic(*rng_));
}

void UtilDB::genRandBatteryData(Sensors::Battery & battery)
{
    std::uniform_real_distribution<float> voltage_battery(BATT_VOLT_LBND, BATT_VOLT_UBND);
    std::uniform_real_distribution<float> current_battery(BATT_CURR_LBND, BATT_CURR_UBND);

    battery.set_voltage(voltage_battery(*rng_));
    battery.set_current(current_battery(*rng_));
}

void UtilDB::genRandWindData(Sensors::Wind & wind_data)
{
    std::uniform_real_distribution<float> speed_wind(SPEED_LBND, SPEED_UBND);
    std::uniform_int_distribution<int>    direction_wind(WIND_DIRECTION_LBND, WIND_DIRECTION_UBND);

    wind_data.set_speed(speed_wind(*rng_));
    wind_data.set_direction(direction_wind(*rng_));
}

void UtilDB::genRandPathData(Sensors::Path & path_data)
{
    std::uniform_real_distribution<float> latitude_path(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> longitude_path(LON_LBND, LON_UBND);

    for (int i = 0; i < NUM_PATH_WAYPOINTS; i++) {
        Polaris::Waypoint * waypoint = path_data.add_waypoints();
        waypoint->set_latitude(latitude_path(*rng_));
        waypoint->set_longitude(longitude_path(*rng_));
    }
}

void UtilDB::genGlobalPathData(Polaris::GlobalPath & global_path_data)  //
{
    std::uniform_real_distribution<float> latitude_path(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> longitude_path(LON_LBND, LON_UBND);
    (void)rng_;
    for (int i = 0; i < NUM_PATH_WAYPOINTS; i++) {
        Polaris::Waypoint * waypoint = global_path_data.add_waypoints();
        waypoint->set_latitude(latitude_path(*rng_));  // this needs a float
        waypoint->set_longitude(longitude_path(*rng_));
    }
    global_path_data.num_waypoints();
}
