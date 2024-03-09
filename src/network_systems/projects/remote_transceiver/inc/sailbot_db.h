#pragma once

#include <google/protobuf/repeated_field.h>

#include <mongocxx/collection.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/pool.hpp>

#include "sensors.pb.h"
#include "waypoint.pb.h"

// BSON document formats (from: https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1907589126/Database+Schemas):

// GPS
// {
//   latitude: decimal,
//   longitude: decimal,
//   speed: decimal,
//   heading: decimal,
//   timestamp: <year - 2000>-<month>-<day> <hour>:<minute>:<second>
// }

// Global Path
// {
//   waypoints: [
//     {
//       latitude: decimal,
//       longitude: decimal
//     }
//   ],
//   timestamp: <year - 2000>-<month>-<day> <hour>:<minute>:<second>
// }

// Local Path
// {
//   waypoints: [
//     {
//       latitude: decimal,
//       longitude: decimal
//     }
//   ],
//   timestamp: <year - 2000>-<month>-<day> <hour>:<minute>:<second>
// }

// AIS Ships
// {
//   ships: [
//     {
//       id: Number,
//       latitude: decimal,
//       longitude: decimal,
//       cog: decimal,
//       rot: decimal,
//       sog: decimal,
//       width: decimal,
//       length: decimal
//     }
//   ],
//   timestamp: <year - 2000>-<month>-<day> <hour>:<minute>:<second>
// }

// Generic Sensors
// {
//   genericSensors: [
//     {
//       id: integer
//       data: long
//     }
//   ],
//   timestamp: <year - 2000>-<month>-<day> <hour>:<minute>:<second>
// }

// Wind Sensors
// {
//   windSensors: [
//     {
//       speed: decimal,
//       direction: number
//     }
//   ],
//   timestamp: <year - 2000>-<month>-<day> <hour>:<minute>:<second>
// }

// Batteries
// {
//   batteries: [
//     {
//       voltage: decimal,
//       current: decimal
//     }
//   ],
//   timestamp: <year - 2000>-<month>-<day> <hour>:<minute>:<second>
// }

const std::string COLLECTION_AIS_SHIPS    = "ais_ships";
const std::string COLLECTION_BATTERIES    = "batteries";
const std::string COLLECTION_DATA_SENSORS = "data_sensors";
const std::string COLLECTION_GPS          = "gps";
const std::string COLLECTION_WIND_SENSORS = "wind_sensors";
const std::string COLLECTION_LOCAL_PATH   = "local_path";
const std::string MONGODB_CONN_STR        = "mongodb://localhost:27017";

template <typename T>
using ProtoList = google::protobuf::RepeatedPtrField<T>;
using DocVal    = bsoncxx::document::view_or_value;

/**
 * Thread-safe class that encapsulates a Sailbot MongoDB database
 *
 */
class SailbotDB
{
public:
    /**
     * Structure to represent metadata associated with a received Iridium message
     * contains only the Iridium header values - the actual payload needs to be further parsed
     */
    struct RcvdMsgInfo
    {
        float       lat_;        // Transmission latitude
        float       lon_;        // Transmission longitude
        uint32_t    cep_;        // Transmission accuracy (km)
        std::string timestamp_;  // Transmission time (<year - 2000>-<month>-<day> <hour>:<minute>:<second>)

        /**
         * @brief overload stream operator
         * When a RcvdMsgInfo class is output to a stream, format the output
         * example: std::cout << RcvdMsgInfo_inst << std::endl; will give us a string formatted as such below
         */
        friend std::ostream & operator<<(std::ostream & os, const RcvdMsgInfo & info)
        {
            os << "Latitude: " << info.lat_ << "\n"
               << "Longitude: " << info.lon_ << "\n"
               << "Accuracy (km): " << info.cep_ << "\n"
               << "Timestamp: " << info.timestamp_;
            return os;
        }
    };

    /**
    * @brief Construct a new SailbotDB object
    *
    * @param db_name          name of desired database
    * @param mongodb_conn_str URL for mongodb database (ex. mongodb://localhost:27017)
    */
    explicit SailbotDB(const std::string & db_name, const std::string & mongodb_conn_str);

    /**
     * @brief Format and print a document in the DB
     *
     * @param doc document value
     */
    static void printDoc(const DocVal & doc);

    /**
     * @brief Ping the connected database to see if the connection succeeded
     *
     * @return true  if ping is successful
     * @return false if ping fails
     */
    bool testConnection();

    /**
     * @brief Write new sensor data to the database
     *
     * @param sensors_pb Protobuf Sensors object
     * @param new_info   Transmission information for the new data
     *
     * @return true  if successful
     * @return false on failure
     */
    bool storeNewSensors(const Polaris::Sensors & sensors_pb, RcvdMsgInfo new_info);

protected:
    const std::string               db_name_;  // Name of the database
    std::unique_ptr<mongocxx::pool> pool_;     // pool of clients for thread safety

private:
    static mongocxx::instance inst_;  // MongoDB instance (must be present - there can only ever be one)

    /**
     * @brief Write GPS data to the database
     *
     * @param gps_pb    Protobuf GPS object
     * @param timestamp transmission time <year - 2000>-<month>-<day> <hour>:<minute>:<second>
     * @param client    mongocxx::client instance for the current thread
     *
     * @return true  if successful
     * @return false on failure
     */
    bool storeGps(const Polaris::Sensors::Gps & gps_pb, const std::string & timestamp, mongocxx::client & client);

    /**
     * @brief Write AIS data to the database
     *
     * @param ais_ships_pb Protobuf list of AIS objects, where the size of the list is the number of ships
     * @param timestamp    transmission time <year - 2000>-<month>-<day> <hour>:<minute>:<second>
     * @param client       mongocxx::client instance for the current thread

     * @return true  if successful
     * @return false on failure
     */
    bool storeAis(
      const ProtoList<Polaris::Sensors::Ais> & ais_ships_pb, const std::string & timestamp, mongocxx::client & client);

    /**
     * @brief Write path sensor data to the database
     *
     * @param generic_pb Protobuf list of path sensor objects, where the size of the list is the number of path sensors
     * @param timestamp  transmission time <year - 2000>-<month>-<day> <hour>:<minute>:<second>
     * @param client     mongocxx::client instance for the current thread

     * @return true  if successful
     * @return false on failure
     */
    bool storePathSensors(
      const Polaris::Sensors::Path & local_path_pb, const std::string & timestamp, mongocxx::client & client);

    /**
    * @brief Adds generic sensors to the database
    *
    * @param generic_pb Protobuf list of generic sensor objects, where the size of the list is the number of sensors
    * @param timestamp  transmission time <year - 2000>-<month>-<day> <hour>:<minute>:<second>
    * @param client     mongocxx::client instance for the current thread
    *
    * @return True if sensor is added, false otherwise
    */
    bool storeGenericSensors(
      const ProtoList<Polaris::Sensors::Generic> & generic_pb, const std::string & timestamp,
      mongocxx::client & client);

    /**
    * @brief Adds a battery sensors to the database
    *
    * @param generic_pb Protobuf list of battery objects
    * @param timestamp  transmission time <year - 2000>-<month>-<day> <hour>:<minute>:<second>
    * @param client     mongocxx::client instance for the current thread
    *
    * @return True if sensor is added, false otherwise
    */
    bool storeBatteries(
      const ProtoList<Polaris::Sensors::Battery> & battery_pb, const std::string & timestamp,
      mongocxx::client & client);

    /**
    * @brief Adds a wind sensor to the database flow
    *
    * @param generic_pb Protobuf list of wind sensor objects
    * @param timestamp  transmission time <year - 2000>-<month>-<day> <hour>:<minute>:<second>
    * @param client     mongocxx::client instance for the current thread
    *
    * @return True if sensor is added, false otherwise
    */
    bool storeWindSensors(
      const ProtoList<Polaris::Sensors::Wind> & wind_pb, const std::string & timestamp, mongocxx::client & client);
};
