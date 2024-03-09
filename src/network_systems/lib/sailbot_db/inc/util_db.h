#pragma once

#include <random>
#include <span>

#include "sailbot_db.h"
#include "sensors.pb.h"
#include "utils/utils.h"

class UtilDB : public SailbotDB
{
public:
    static constexpr int NUM_AIS_SHIPS       = 15;  // arbitrary number
    static constexpr int NUM_GENERIC_SENSORS = 5;   // arbitrary number
    static constexpr int NUM_PATH_WAYPOINTS  = 5;   // arbitrary number

    /**
     * @brief Construct a UtilDB, which has debug utilities for SailbotDB
     *
     * @param db_name
     * @param mongodb_conn_str
     * @param rng
     */
    UtilDB(const std::string & db_name, const std::string & mongodb_conn_str, std::shared_ptr<std::mt19937> rng);

    /**
     * @brief Delete all documents in all collections
     */
    void cleanDB();

    /**
    * @brief Generate random data for all sensors
    *
    * @return Sensors object
    */
    Polaris::Sensors genRandSensors();

    /**
     * @return timestamp for the current time
     */
    static std::tm getTimestamp();

    /**
    * @brief Generate random sensors and Iridium msg info
    *
    * @param tm Timestamp returned by getTimestamp() (with any modifications made to it)
    * @return std::pair<Sensors, SailbotDB::RcvdMsgInfo>
    */
    std::pair<Polaris::Sensors, SailbotDB::RcvdMsgInfo> genRandData(const std::tm & tm);

    /**
    * @brief Query the database and check that the sensor and message are correct
    *
    * @param expected_sensors
    * @param expected_msg_info
    */
    bool verifyDBWrite(
      std::span<Polaris::Sensors> expected_sensors, std::span<SailbotDB::RcvdMsgInfo> expected_msg_info);

    /**
     * @brief Dump and check all sensors and timestamps from the database
     *
     * @param tracker           FailureTracker that gets if any unexpected results are dumped
     * @param expected_num_docs Expected number of documents. tracker is updated if there's a mismatch
     * @return std::pair{Vector of dumped Sensors, Vector of dumped timestamps}
     */
    std::pair<std::vector<Polaris::Sensors>, std::vector<std::string>> dumpSensors(
      utils::FailTracker & tracker, size_t expected_num_docs = 1);

private:
    std::shared_ptr<std::mt19937> rng_;  // random number generator

    /**
    * @brief generate random GPS data
    *
    * @param gps_data GPS data to modify
    */
    void genRandGpsData(Polaris::Sensors::Gps & gps_data);

    /**
    * @brief generate random ais ships data
    *
    * @param ais_ship AIS ship data to modify
    */
    void genRandAisData(Polaris::Sensors::Ais & ais_ship);

    /**
    * @brief generate random generic sensor data
    *
    * @param generic_sensor Generic sensor data to modify
    */
    void genRandGenericSensorData(Polaris::Sensors::Generic & generic_sensor);

    /**
    * @brief generate random battery data
    *
    * @param battery battery data to modify
    */
    void genRandBatteryData(Polaris::Sensors::Battery & battery);

    /**
    * @brief generate random wind sensors data
    *
    * @param wind_data Wind sensor data to modify
    */
    void genRandWindData(Polaris::Sensors::Wind & wind_data);

    /**
    * @brief generate random path data
    *
    * @param path_data Path data to modify
    */
    void genRandPathData(Polaris::Sensors::Path & path_data);
};
