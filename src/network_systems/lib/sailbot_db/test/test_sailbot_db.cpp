#include <gtest/gtest.h>

#include "mongocxx/client.hpp"
#include "sailbot_db.h"
#include "util_db.h"

using Polaris::GlobalPath;
using Polaris::Sensors;

static std::random_device g_rd        = std::random_device();  // random number sampler
static uint32_t           g_rand_seed = g_rd();                // seed used for random number generation
static std::mt19937       g_mt(g_rand_seed);                   // initialize random number generator with seed
static UtilDB             g_test_db("test", MONGODB_CONN_STR, std::make_shared<std::mt19937>(g_mt));
class TestSailbotDB : public ::testing::Test
{
protected:
    TestSailbotDB() { g_test_db.cleanDB(); }
    ~TestSailbotDB() {}
};

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
    auto [rand_sensors, rand_info] = g_test_db.genRandData(UtilDB::getTimestamp());
    ASSERT_TRUE(g_test_db.storeNewSensors(rand_sensors, rand_info));

    std::array<Sensors, 1>                expected_sensors = {rand_sensors};
    std::array<SailbotDB::RcvdMsgInfo, 1> expected_info    = {rand_info};

    EXPECT_TRUE(g_test_db.verifyDBWrite(expected_sensors, expected_info));
}

TEST_F(TestSailbotDB, TestStoreGlobalPath)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    auto [global_path_data, global_path_timestamp] = g_test_db.genGlobalData(UtilDB::getTimestamp());
    ASSERT_TRUE(g_test_db.storeNewGlobalPath(global_path_data, global_path_timestamp));

    std::array<GlobalPath, 1>  expected_global_path_data      = {global_path_data};
    std::array<std::string, 1> expected_global_path_timestamp = {global_path_timestamp};

    EXPECT_TRUE(g_test_db.verifyDBWrite_GlobalPath(expected_global_path_data, expected_global_path_timestamp));
}
