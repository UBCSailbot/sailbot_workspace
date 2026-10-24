#pragma once

#include <google/protobuf/repeated_field.h>

#include <cstdint>
#include <mongocxx/collection.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/pool.hpp>
#include <optional>
#include <string>
#include <vector>

#include "global_path.pb.h"
#include "sensors.pb.h"
#include "waypoint.pb.h"

const std::string COLLECTION_AIS_SHIPS        = "ais_ships";
const std::string COLLECTION_BATTERIES        = "batteries";
const std::string COLLECTION_TEMP_SENSORS     = "temp_sensors";
const std::string COLLECTION_PH_SENSORS       = "ph_sensors";
const std::string COLLECTION_SALINITY_SENSORS = "salinity_sensors";
const std::string COLLECTION_GPS              = "gps";
const std::string COLLECTION_WIND_SENSORS     = "wind_sensors";
const std::string COLLECTION_LOCAL_PATH       = "local_path";
const std::string COLLECTION_GLOBAL_PATH      = "global_path";
const std::string COLLECTION_IRIDIUM_RESPONSE = "iridium_response";
const std::string COLLECTION_VOYAGES          = "voyages";
const std::string COLLECTION_ACTIVE_VOYAGE    = "active_voyage";
const std::string MONGODB_ATLAS_HOST          = "dev.khxge.mongodb.net";
const std::string MONGODB_ATLAS_USER          = "software";

// The collections that storeNewSensors writes one document to for every upload
const std::vector<std::string> SENSOR_COLLECTIONS = {
  COLLECTION_GPS,          COLLECTION_AIS_SHIPS,        COLLECTION_TEMP_SENSORS,
  COLLECTION_PH_SENSORS,   COLLECTION_SALINITY_SENSORS, COLLECTION_BATTERIES,
  COLLECTION_WIND_SENSORS, COLLECTION_LOCAL_PATH};

template <typename T>
using ProtoList = google::protobuf::RepeatedPtrField<T>;

template <typename T>
using ProtoPrimitiveList = google::protobuf::RepeatedField<T>;

using DocVal = bsoncxx::document::view_or_value;

template <typename Element>
std::optional<std::string> bsonString(const Element & element)
{
    constexpr std::uint8_t BSON_STRING_TYPE = 0x02;
    if (!element || static_cast<std::uint8_t>(element.type()) != BSON_STRING_TYPE) {
        return std::nullopt;
    }
    if constexpr (requires { element.get_string(); }) {
        const auto text = element.get_string().value;
        return std::string(text.data(), text.size());
    } else {
        const auto text = element.get_utf8().value;
        return std::string(text.data(), text.size());
    }
}

class SailbotDB
{
public:
    struct RcvdMsgInfo
    {
        float                 lat_;
        float                 lon_;
        uint32_t              cep_;
        std::string           timestamp_;
        friend std::ostream & operator<<(std::ostream & os, const RcvdMsgInfo & info);
    };
    static const std::string & MONGODB_CONN_STR();
    static std::string         atlasConnStr(const std::string & password);
    static std::string         mkTimestamp(const std::tm & tm);
    static std::string         normalizeTimestamp(const std::string & timestamp);
    SailbotDB(const std::string & db_name, const std::string & mongodb_conn_str = MONGODB_CONN_STR());
    void        printDoc(const DocVal & doc);
    bool        testConnection();
    bool        storeNewSensors(const Polaris::Sensors & sensors_pb, RcvdMsgInfo new_info);
    std::string startNewVoyage();
    std::string getActiveVoyage();
    bool        storeNewGlobalPath(const Polaris::GlobalPath & global_pb, const std::string & timestamp);
    bool        storeIridiumResponse(
      const std::string & response, const std::string & error, const std::string & message,
      const std::string & timestamp);

protected:
    std::string                     db_name_;
    std::unique_ptr<mongocxx::pool> pool_;
    std::string                     generateVoyageName();
    void                            insertVoyage(const std::string & name, const std::string & created_by);
    void                            activateVoyage(const std::string & name);
    void                            renameActiveVoyage(const std::string & old_name, const std::string & new_name);

private:
    bool storeGps(
      const Polaris::Sensors::Gps & gps_pb, const std::string & timestamp, const std::string & voyage,
      mongocxx::client & client);
    bool storeAis(
      const ProtoList<Polaris::Sensors::Ais> & ais_ships_pb, const std::string & timestamp, const std::string & voyage,
      mongocxx::client & client);
    bool storeTempSensors(
      const ProtoPrimitiveList<float> & temp_pb, const std::string & timestamp, const std::string & voyage,
      mongocxx::client & client);
    bool storePhSensors(
      const ProtoPrimitiveList<float> & ph_pb, const std::string & timestamp, const std::string & voyage,
      mongocxx::client & client);
    bool storeSalinitySensors(
      const ProtoPrimitiveList<float> & salinity_pb, const std::string & timestamp, const std::string & voyage,
      mongocxx::client & client);
    bool storeBatteries(
      const ProtoList<Polaris::Sensors::Battery> & battery_pb, const std::string & timestamp,
      const std::string & voyage, mongocxx::client & client);
    bool storeWindSensors(
      const ProtoList<Polaris::Sensors::Wind> & wind_pb, const std::string & timestamp, const std::string & voyage,
      mongocxx::client & client);
    bool storePathSensors(
      const Polaris::Sensors::Path & local_path_pb, const std::string & timestamp, const std::string & voyage,
      mongocxx::client & client);
    bool storeNewGlobalPath(
      const Polaris::GlobalPath & global_path_pb, const std::string & timestamp, mongocxx::client & client);
    bool storeIridiumResponse(
      const std::string & response, const std::string & error, const std::string & message,
      const std::string & timestamp, mongocxx::client & client);
};
