/* IMPORTANT: Make sure only one instance of network_systems/scripts/run_virtual_iridium.sh is running */

#include <gtest/gtest.h>

#include <boost/system/system_error.hpp>

#include "cmn_hdrs/shared_constants.h"
#include "local_transceiver.h"
#include "sensors.pb.h"

class TestLocalTransceiver : public ::testing::Test
{
protected:
    TestLocalTransceiver()
    {
        try {
            lcl_trns_ = new LocalTransceiver(LOCAL_TRANSCEIVER_TEST_PORT, SATELLITE_BAUD_RATE);
        } catch (boost::system::system_error & /**/) {
            std::cerr << "Failed to create Local Transceiver for tests, is only one instance of: \""
                      << RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH << "\" running?" << std::endl;
        }
    }
    ~TestLocalTransceiver() override
    {
        lcl_trns_->stop();
        delete lcl_trns_;
    }

    LocalTransceiver * lcl_trns_;
};

/**
 * @brief Verify debugSendTest sends something to the terminal
 */
TEST_F(TestLocalTransceiver, debugSendTest)
{
    //std::string testDebug = "showsUp";
    //lcl_trns_->debugSend(testDebug);
}

/**
 * @brief Send a binary string to virtual_iridium and verify it is received
 *        Uses gps custom interface
 */
TEST_F(TestLocalTransceiver, sendGpsTest)
{
    constexpr float holder = 14.3;

    custom_interfaces::msg::GPS gps;
    gps.heading.set__heading(holder);
    gps.lat_lon.set__latitude(holder);
    gps.lat_lon.set__longitude(holder);
    gps.speed.set__speed(holder);
    lcl_trns_->updateSensor(gps);
    lcl_trns_->send();
}
