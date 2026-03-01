#include "sailbot_db.h"

#include <bsoncxx/builder/stream/array.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/json.hpp>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mongocxx/client.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/instance.hpp>
#include <sstream>

#include "global_path.pb.h"
#include "sensors.pb.h"
#include "waypoint.pb.h"

namespace bstream = bsoncxx::builder::stream;
using Polaris::GlobalPath;
using Polaris::Sensors;

mongocxx::instance SailbotDB::inst_{};  // statically initialize instance

// PUBLIC

std::ostream & operator<<(std::ostream & os, const SailbotDB::RcvdMsgInfo & info)
{
    os << "Latitude: " << info.lat_ << "\n"
       << "Longitude: " << info.lon_ << "\n"
       << "Accuracy (km): " << info.cep_ << "\n"
       << "Timestamp: " << info.timestamp_;
    return os;
}

const std::string & SailbotDB::MONGODB_CONN_STR()
{
    static const std::string conn_str = [] {
        const char * pwd = std::getenv("MONGODB_PASSWORD");  //NOLINT(concurrency-mt-unsafe)
        if (pwd == nullptr || std::strcmp(pwd, "placeholder") == 0) {
            const char * github_uri = std::getenv("MONGODB_URI");  //NOLINT(concurrency-mt-unsafe)
            if (github_uri != nullptr) {
                return std::string(github_uri);  // If running in GitHub Actions, use the full URI from secrets
            }
            printf(
              "MongoDB password not found (must be inputted into test.sh): Running MongoDB tests locally \n");  // If running tests locally
            return std::string("mongodb://localhost:27017");
        }
        return "mongodb+srv://software:" + std::string(pwd) + "@dev.khxge.mongodb.net/?appName=dev";
    }();
    return conn_str;
}

std::string SailbotDB::mkTimestamp(const std::tm & tm)
{
    constexpr int     YEAR_OFFSET  = 1900;  // tm_year is years since 1900
    constexpr int     YEAR_MODULUS = 100;   // used to extract the last two digits of the year
    std::stringstream tm_ss;
    tm_ss << std::setfill('0') << std::setw(2)
          << (tm.tm_year + YEAR_OFFSET) % YEAR_MODULUS  // format last 2 digits of year (e.g., 2025 → 25)
          << "-" << std::setfill('0') << std::setw(2) << (tm.tm_mon + 1)  // months are 0-based (0 = January)
          << "-" << std::setfill('0') << std::setw(2) << tm.tm_mday << " " << std::setfill('0') << std::setw(2)  // hour
          << tm.tm_hour << ":" << std::setfill('0') << std::setw(2) << tm.tm_min << ":" << std::setfill('0')  // minute
          << std::setw(2) << tm.tm_sec;                                                                       // second
    return tm_ss.str();
}

SailbotDB::SailbotDB(const std::string & db_name, const std::string & mongodb_conn_str) : db_name_(db_name)
{
    mongocxx::uri uri = mongocxx::uri{mongodb_conn_str};
    pool_             = std::make_unique<mongocxx::pool>(uri);
}

void SailbotDB::printDoc(const DocVal & doc) { std::cout << bsoncxx::to_json(doc.view()) << std::endl; }

bool SailbotDB::testConnection()
{
    const DocVal          ping_cmd = bstream::document{} << "ping" << 1 << bstream::finalize;
    mongocxx::pool::entry entry    = pool_->acquire();
    mongocxx::database    db       = (*entry)[db_name_];
    try {
        // Ping the database.
        db.run_command(ping_cmd.view());
        return true;
    } catch (const std::exception & e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return false;
    }
}

bool SailbotDB::storeNewSensors(const Sensors & sensors_pb, RcvdMsgInfo new_info)
{
    // Only using timestamp info for now, may use other fields in the future
    const std::string &   timestamp = new_info.timestamp_;
    mongocxx::pool::entry entry     = pool_->acquire();
    return storeGps(sensors_pb.gps(), timestamp, *entry) && storeAis(sensors_pb.ais_ships(), timestamp, *entry) &&
           //    storeGenericSensors(sensors_pb.data_sensors(), timestamp, *entry) && // * Generic sensors are replaced by Temp, Salinity, pH, Pressure
           storeTempSensors(sensors_pb.temp_sensors(), timestamp, *entry) &&
           storePhSensors(sensors_pb.ph_sensors(), timestamp, *entry) &&
           storeSalinitySensors(sensors_pb.salinity_sensors(), timestamp, *entry) &&
           //    storePressureSensors(sensors_pb.pressure_sensors(), timestamp, *entry) && // * We aren't using pH Sensors as of Nov 2025
           storeBatteries(sensors_pb.batteries(), timestamp, *entry) &&
           storeWindSensors(sensors_pb.wind_sensors(), timestamp, *entry) &&
           storePathSensors(sensors_pb.local_path_data(), timestamp, *entry);
}

// END PUBLIC

bool SailbotDB::storeNewGlobalPath(const GlobalPath & global_pb, const std::string & timestamp)
{
    mongocxx::pool::entry entry = pool_->acquire();
    return storeNewGlobalPath(global_pb, timestamp, *entry);
}

bool SailbotDB::storeIridiumResponse(
  const std::string & response, const std::string & error, const std::string & message, const std::string & timestamp)
{
    mongocxx::pool::entry entry = pool_->acquire();
    return storeIridiumResponse(response, error, message, timestamp, *entry);
}

// PRIVATE

bool SailbotDB::storeGps(const Sensors::Gps & gps_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database   db       = client[db_name_];
    mongocxx::collection gps_coll = db[COLLECTION_GPS];
    DocVal gps_doc = bstream::document{} << "latitude" << gps_pb.latitude() << "longitude" << gps_pb.longitude()
                                         << "speed" << gps_pb.speed() << "heading" << gps_pb.heading() << "timestamp"
                                         << timestamp << bstream::finalize;

    return static_cast<bool>(gps_coll.insert_one(gps_doc.view()));
}

bool SailbotDB::storeAis(
  const ProtoList<Sensors::Ais> & ais_ships_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database   db       = client[db_name_];
    mongocxx::collection ais_coll = db[COLLECTION_AIS_SHIPS];
    bstream::document    doc_builder{};
    auto                 ais_ships_doc_arr = doc_builder << "ships" << bstream::open_array;
    for (const Sensors::Ais & ais_ship : ais_ships_pb) {
        // The BSON spec does not allow unsigned integers (throws exception), so cast our uint32s to sint64s
        ais_ships_doc_arr = ais_ships_doc_arr
                            << bstream::open_document << "id" << static_cast<int64_t>(ais_ship.id()) << "latitude"
                            << ais_ship.latitude() << "longitude" << ais_ship.longitude() << "sog" << ais_ship.sog()
                            << "cog" << ais_ship.cog() << "rot" << ais_ship.rot() << "width" << ais_ship.width()
                            << "length" << ais_ship.length() << bstream::close_document;
    }
    DocVal ais_ships_doc = ais_ships_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
    return static_cast<bool>(ais_coll.insert_one(ais_ships_doc.view()));
}

// bool SailbotDB::storeGenericSensors(
//   const ProtoList<Sensors::Generic> & generic_pb, const std::string & timestamp, mongocxx::client & client)
// {
//     mongocxx::database   db           = client[db_name_];
//     mongocxx::collection generic_coll = db[COLLECTION_DATA_SENSORS];
//     bstream::document    doc_builder{};
//     auto                 generic_doc_arr = doc_builder << "genericSensors" << bstream::open_array;
//     for (const Sensors::Generic & generic : generic_pb) {
//         generic_doc_arr = generic_doc_arr << bstream::open_document << "id" << static_cast<int64_t>(generic.id())
//                                           << "data" << static_cast<int64_t>(generic.data()) << bstream::close_document;
//     }
//     DocVal generic_doc = generic_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
//     return static_cast<bool>(generic_coll.insert_one(generic_doc.view()));
// }

bool SailbotDB::storeTempSensors(
  const ProtoPrimitiveList<float> & temp_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database   db        = client[db_name_];
    mongocxx::collection temp_coll = db[COLLECTION_TEMP_SENSORS];
    bstream::document    doc_builder{};
    auto                 temp_doc_arr = doc_builder << "tempSensors" << bstream::open_array;
    for (const float & temp_sensor : temp_pb) {
        temp_doc_arr = temp_doc_arr << bstream::open_document << "temperature" << temp_sensor
                                    << bstream::close_document;
    }
    DocVal temp_doc = temp_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
    return static_cast<bool>(temp_coll.insert_one(temp_doc.view()));
}

bool SailbotDB::storePhSensors(
  const ProtoPrimitiveList<float> & ph_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database   db      = client[db_name_];
    mongocxx::collection ph_coll = db[COLLECTION_PH_SENSORS];
    bstream::document    doc_builder{};
    auto                 ph_doc_arr = doc_builder << "phSensors" << bstream::open_array;
    for (const float & ph_sensor : ph_pb) {
        ph_doc_arr = ph_doc_arr << bstream::open_document << "ph" << ph_sensor << bstream::close_document;
    }
    DocVal ph_doc = ph_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
    return static_cast<bool>(ph_coll.insert_one(ph_doc.view()));
}

bool SailbotDB::storeSalinitySensors(
  const ProtoPrimitiveList<float> & salinity_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database   db            = client[db_name_];
    mongocxx::collection salinity_coll = db[COLLECTION_SALINITY_SENSORS];
    bstream::document    doc_builder{};
    auto                 salinity_doc_arr = doc_builder << "salinitySensors" << bstream::open_array;
    for (const float & salinity_sensor : salinity_pb) {
        salinity_doc_arr = salinity_doc_arr << bstream::open_document << "salinity" << salinity_sensor
                                            << bstream::close_document;
    }
    DocVal salinity_doc = salinity_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
    return static_cast<bool>(salinity_coll.insert_one(salinity_doc.view()));
}

// bool SailbotDB::storePressureSensors(
//   const ProtoPrimitiveList<float> & pressure_pb, const std::string & timestamp, mongocxx::client & client)
// {
//     mongocxx::database   db            = client[db_name_];
//     mongocxx::collection pressure_coll = db[COLLECTION_PRESSURE_SENSORS];
//     bstream::document    doc_builder{};
//     auto                 pressure_doc_arr = doc_builder << "pressureSensors" << bstream::open_array;
//     for (const float & pressure_sensor : pressure_pb) {
//         pressure_doc_arr = pressure_doc_arr << bstream::open_document << "pressure" << pressure_sensor
//                                             << bstream::close_document;
//     }
//     DocVal pressure_doc = pressure_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
//     return static_cast<bool>(pressure_coll.insert_one(pressure_doc.view()));
// }

bool SailbotDB::storeBatteries(
  const ProtoList<Sensors::Battery> & battery_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database   db             = client[db_name_];
    mongocxx::collection batteries_coll = db[COLLECTION_BATTERIES];
    bstream::document    doc_builder{};
    auto                 batteries_doc_arr = doc_builder << "batteries" << bstream::open_array;
    for (const Sensors::Battery & battery : battery_pb) {
        batteries_doc_arr = batteries_doc_arr << bstream::open_document << "voltage" << battery.voltage() << "current"
                                              << battery.current() << bstream::close_document;
    }
    DocVal batteries_doc = batteries_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
    return static_cast<bool>(batteries_coll.insert_one(batteries_doc.view()));
}

bool SailbotDB::storeWindSensors(
  const ProtoList<Sensors::Wind> & wind_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database   db        = client[db_name_];
    mongocxx::collection wind_coll = db[COLLECTION_WIND_SENSORS];
    bstream::document    doc_builder{};
    auto                 wind_doc_arr = doc_builder << "windSensors" << bstream::open_array;
    for (const Sensors::Wind & wind_sensor : wind_pb) {
        wind_doc_arr = wind_doc_arr << bstream::open_document << "speed" << wind_sensor.speed() << "direction"
                                    << static_cast<int16_t>(wind_sensor.direction()) << bstream::close_document;
    }
    DocVal wind_doc = wind_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
    return static_cast<bool>(wind_coll.insert_one(wind_doc.view()));
}

bool SailbotDB::storePathSensors(
  const Sensors::Path & local_path_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database           db              = client[db_name_];
    mongocxx::collection         local_path_coll = db[COLLECTION_LOCAL_PATH];
    bstream::document            doc_builder{};
    auto                         local_path_doc_arr = doc_builder << "waypoints" << bstream::open_array;
    ProtoList<Polaris::Waypoint> waypoints          = local_path_pb.waypoints();
    for (const Polaris::Waypoint & waypoint : waypoints) {
        local_path_doc_arr = local_path_doc_arr << bstream::open_document << "latitude" << waypoint.latitude()
                                                << "longitude" << waypoint.longitude() << bstream::close_document;
    }
    DocVal local_path_doc = local_path_doc_arr << bstream::close_array << "timestamp" << timestamp << bstream::finalize;
    return static_cast<bool>(local_path_coll.insert_one(local_path_doc.view()));
}

bool SailbotDB::storeNewGlobalPath(
  const Polaris::GlobalPath & global_path_pb, const std::string & timestamp, mongocxx::client & client)
{
    mongocxx::database           db               = client[db_name_];
    mongocxx::collection         global_path_coll = db[COLLECTION_GLOBAL_PATH];
    bstream::document            doc_builder{};
    auto                         global_path_doc_arr = doc_builder << "waypoints" << bstream::open_array;
    ProtoList<Polaris::Waypoint> waypoints           = global_path_pb.waypoints();
    for (const Polaris::Waypoint & waypoint : waypoints) {
        global_path_doc_arr = global_path_doc_arr << bstream::open_document << "latitude" << waypoint.latitude()
                                                  << "longitude" << waypoint.longitude() << bstream::close_document;
    }
    DocVal global_path_doc = global_path_doc_arr << bstream::close_array << "timestamp" << timestamp
                                                 << bstream::finalize;
    return static_cast<bool>(global_path_coll.insert_one(global_path_doc.view()));
}

bool SailbotDB::storeIridiumResponse(
  const std::string & response, const std::string & error, const std::string & message, const std::string & timestamp,
  mongocxx::client & client)
{
    mongocxx::database   db                    = client[db_name_];
    mongocxx::collection iridium_response_coll = db[COLLECTION_IRIDIUM_RESPONSE];

    DocVal iridium_response_doc = bstream::document{} << "response" << response << "error" << error << "timestamp"
                                                      << timestamp << "message" << message << bstream::finalize;

    return static_cast<bool>(iridium_response_coll.insert_one(iridium_response_doc.view()));
}

// END PRIVATE
