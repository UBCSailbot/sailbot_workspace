#include <errno.h>
#include <fcntl.h>
#include <gtest/gtest.h>

#include <cstring>

#include "can_frame_parser.h"
#include "can_transceiver.h"
#include "cmn_hdrs/shared_constants.h"

namespace msg = custom_interfaces::msg;

constexpr CAN_FP::RawDataBuf GARBAGE_DATA = []() constexpr
{
    CAN_FP::RawDataBuf buf;
    for (uint8_t & entry : buf) {
        entry = 0xFF;  // NOLINT(readability-magic-numbers)
    }
    return buf;
}
();

/**
 * @brief Test ROS<->CAN translations
 *
 */
class TestCanFrameParser : public ::testing::Test
{
protected:
    TestCanFrameParser() {}
    ~TestCanFrameParser() override {}
};

/**
 * @brief Test ROS<->CAN Battery translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, BatteryTestValid)
{
    constexpr std::array<float, NUM_BATTERIES>   expected_volts{12.5, 10.6};
    constexpr std::array<float, NUM_BATTERIES>   expected_currs{2.5, -1.0};  // negative currents are valid
    constexpr std::array<int16_t, NUM_BATTERIES> expected_raw_volts{1250, 1060};
    constexpr std::array<int16_t, NUM_BATTERIES> expected_raw_expected_currs{250, -100};

    for (size_t i = 0; i < NUM_BATTERIES; i++) {
        auto optId = CAN_FP::Battery::rosIdxToCanId(i);

        ASSERT_TRUE(optId.has_value());

        CAN_FP::CanId      id            = optId.value();
        float              expected_volt = expected_volts[i];
        float              expected_curr = expected_currs[i];
        msg::HelperBattery msg;
        msg.set__voltage(expected_volt);
        msg.set__current(expected_curr);
        CAN_FP::Battery  bat_from_ros = CAN_FP::Battery(msg, id);
        CAN_FP::CanFrame cf           = bat_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::Battery::CAN_BYTE_DLEN_);

        int16_t raw_volt;
        int16_t raw_curr;
        std::memcpy(&raw_volt, cf.data + CAN_FP::Battery::BYTE_OFF_VOLT, sizeof(int16_t));
        std::memcpy(&raw_curr, cf.data + CAN_FP::Battery::BYTE_OFF_CURR, sizeof(int16_t));

        EXPECT_EQ(raw_volt, expected_raw_volts[i]);
        EXPECT_EQ(raw_curr, expected_raw_expected_currs[i]);

        CAN_FP::Battery bat_from_can = CAN_FP::Battery(cf);

        EXPECT_EQ(bat_from_can.id_, id);

        msg::HelperBattery msg_from_bat = bat_from_can.toRosMsg();

        EXPECT_DOUBLE_EQ(msg_from_bat.voltage, expected_volt);
        EXPECT_DOUBLE_EQ(msg_from_bat.current, expected_curr);
    }
}

/**
 * @brief Test the behavior of the Battery class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestBatteryInvalid)
{
    auto optId = CAN_FP::Battery::rosIdxToCanId(NUM_BATTERIES);
    EXPECT_FALSE(optId.has_value());

    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::Battery tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<float> invalid_volts{BATT_VOLT_LBND - 1, BATT_VOLT_UBND + 1};
    std::vector<float> invalid_currs{BATT_CURR_LBND - 1, BATT_CURR_UBND + 1};

    optId = CAN_FP::Battery::rosIdxToCanId(0);
    ASSERT_TRUE(optId.has_value());

    CAN_FP::CanId      valid_id = optId.value();
    msg::HelperBattery msg;

    // Set a valid current for this portion
    for (float invalid_volt : invalid_volts) {
        msg.set__voltage(invalid_volt);
        msg.set__current(BATT_CURR_LBND);

        EXPECT_THROW(CAN_FP::Battery tmp(msg, valid_id), std::out_of_range);
    };

    // Set a valid voltage for this portion
    for (float invalid_curr : invalid_currs) {
        msg.set__voltage(BATT_VOLT_LBND);
        msg.set__current(invalid_curr);

        EXPECT_THROW(CAN_FP::Battery tmp(msg, valid_id), std::out_of_range);
    };

    cf.can_id = static_cast<canid_t>(CAN_FP::CanId::BMS_P_DATA_FRAME_1);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);

    EXPECT_THROW(CAN_FP::Battery tmp(cf), std::out_of_range);
}

/**
 * @brief Test CanTransceiver using a mock CAN descriptor
 *
 */
class TestCanTransceiver : public ::testing::Test
{
protected:
    static constexpr auto SLEEP_TIME = std::chrono::milliseconds(20);
    CanTransceiver *      canbus_t_;
    int                   fd_;
    TestCanTransceiver()
    {
        fd_       = mockCanFd("/tmp/TestCanTransceiverXXXXXX");
        canbus_t_ = new CanTransceiver(fd_);
    }
    ~TestCanTransceiver() override { delete canbus_t_; }
};

/**
 * @brief Test that callbacks can be properly registered and invoked on desired CanIds
 *
 */
TEST_F(TestCanTransceiver, TestNewDataValid)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::BMS_P_DATA_FRAME_1, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::BMS_P_DATA_FRAME_1)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that the CanTransceiver ignores IDs that we don't register callbacks for
 *
 */
TEST_F(TestCanTransceiver, TestNewDataIgnore)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::BMS_P_DATA_FRAME_1, test_cb),
    }});

    // just need a valid and ignored ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::RESERVED)};

    canbus_t_->send(dummy_frame);
    // Since we're writing to the same file we're reading from, we need to reset the seek offset
    // This is NOT necessary in deployment as we won't be using a file to mock it
    lseek(fd_, 0, SEEK_SET);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_FALSE(is_cb_called);
}
