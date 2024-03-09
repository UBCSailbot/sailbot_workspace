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
 * @brief Test ROS<->CAN SailCmd translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, SailCmdTestValid)
{
    constexpr std::uint8_t                 NUM_SAILS = CAN_FP::SailCmd::SAIL_CMD_IDS.size();
    constexpr std::array<float, NUM_SAILS> expected_angles{12, 128};

    for (size_t i = 0; i < NUM_SAILS; i++) {
        CAN_FP::CanId id             = CAN_FP::SailCmd::SAIL_CMD_IDS[i];
        float         expected_angle = expected_angles[i];
        msg::SailCmd  msg;
        msg.set__trim_tab_angle_degrees(expected_angle);
        CAN_FP::SailCmd  sail_from_ros = CAN_FP::SailCmd(msg, id);
        CAN_FP::CanFrame cf            = sail_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::SailCmd::CAN_BYTE_DLEN_);

        int16_t raw_angle;
        std::memcpy(&raw_angle, cf.data + CAN_FP::SailCmd::BYTE_OFF_ANGLE, sizeof(int16_t));

        EXPECT_EQ(raw_angle, expected_angle);

        CAN_FP::SailCmd sail_from_can = CAN_FP::SailCmd(cf);

        EXPECT_EQ(sail_from_can.id_, id);

        msg::SailCmd msg_from_bat = sail_from_can.toRosMsg();

        EXPECT_DOUBLE_EQ(msg_from_bat.trim_tab_angle_degrees, expected_angle);
    }
}

/**
 * @brief Test the behavior of the SailCmd class when given invalid Id values
 *
 */
TEST_F(TestCanFrameParser, TestSailCmdInvalid)
{
    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::SailCmd tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<float> invalid_angles{HEADING_LBND - 1, HEADING_UBND + 1};

    CAN_FP::CanId valid_id = CAN_FP::CanId::SAIL_WSM_CMD_FRAME_1;
    msg::SailCmd  msg;

    for (float invalid_angle : invalid_angles) {
        msg.set__trim_tab_angle_degrees(invalid_angle);

        EXPECT_THROW(CAN_FP::SailCmd tmp(msg, valid_id), std::out_of_range);
    };

    cf.can_id = static_cast<canid_t>(CAN_FP::CanId::SAIL_WSM_CMD_FRAME_1);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);

    EXPECT_THROW(CAN_FP::SailCmd tmp(cf), std::out_of_range);
}

/**
 * @brief Test ROS<->CAN Battery translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, WindSensorTestValid)
{
    constexpr std::uint8_t                     NUM_SENSORS = CAN_FP::WindSensor::WIND_SENSOR_IDS.size();
    constexpr std::array<float, NUM_SENSORS>   expected_speeds{8.9, 3.0};
    constexpr std::array<int16_t, NUM_SENSORS> expected_angles{89, 167};

    for (size_t i = 0; i < NUM_SENSORS; i++) {
        auto optId = CAN_FP::WindSensor::rosIdxToCanId(i);

        ASSERT_TRUE(optId.has_value());

        CAN_FP::CanId    id             = optId.value();
        float            expected_speed = expected_speeds[i];
        int16_t          expected_angle = expected_angles[i];
        msg::WindSensor  msg;
        msg::HelperSpeed speed_msg;

        speed_msg.set__speed(expected_speed);
        msg.set__speed(speed_msg);
        msg.set__direction(expected_angle);

        CAN_FP::WindSensor sensor_from_ros = CAN_FP::WindSensor(msg, id);
        CAN_FP::CanFrame   cf              = sensor_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::WindSensor::CAN_BYTE_DLEN_);

        int16_t raw_speed;
        int16_t raw_angle;
        std::memcpy(&raw_speed, cf.data + CAN_FP::WindSensor::BYTE_OFF_SPEED, sizeof(int16_t));
        std::memcpy(&raw_angle, cf.data + CAN_FP::WindSensor::BYTE_OFF_ANGLE, sizeof(int16_t));
        float converted_speed = static_cast<float>(raw_speed) * 1.852 / 10.0;  //NOLINT

        EXPECT_NEAR(converted_speed, expected_speeds[i], 0.1852);
        EXPECT_EQ(raw_angle, expected_angles[i]);

        CAN_FP::WindSensor sensor_from_can = CAN_FP::WindSensor(cf);

        EXPECT_EQ(sensor_from_can.id_, id);

        msg::WindSensor msg_from_sensor = sensor_from_can.toRosMsg();

        EXPECT_NEAR(msg_from_sensor.speed.speed, expected_speed, 0.1852);
        EXPECT_DOUBLE_EQ(msg_from_sensor.direction, expected_angle);
    }
}

TEST_F(TestCanFrameParser, TestWindSensorInvalid)
{
    auto optId = CAN_FP::WindSensor::rosIdxToCanId(NUM_WIND_SENSORS);
    EXPECT_FALSE(optId.has_value());

    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::WindSensor tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<int16_t> invalid_angles{WIND_DIRECTION_LBND - 1, WIND_DIRECTION_UBND + 1};
    std::vector<float>   invalid_speeds{SPEED_LBND - 1, SPEED_UBND + 1};

    optId = CAN_FP::WindSensor::rosIdxToCanId(0);
    ASSERT_TRUE(optId.has_value());

    CAN_FP::CanId   valid_id = optId.value();
    msg::WindSensor msg;

    // Set a valid speed for this portion
    for (int16_t invalid_angle : invalid_angles) {
        msg.set__direction(invalid_angle);
        msg::HelperSpeed tmp_speed_msg;
        tmp_speed_msg.set__speed(SPEED_LBND);
        msg.set__speed(tmp_speed_msg);

        EXPECT_THROW(CAN_FP::WindSensor tmp(msg, valid_id), std::out_of_range);
    };

    // Set a valid direction for this portion
    for (float invalid_speed : invalid_speeds) {
        msg.set__direction(WIND_DIRECTION_UBND);
        msg::HelperSpeed tmp_speed_msg;
        tmp_speed_msg.set__speed(invalid_speed);
        msg.set__speed(tmp_speed_msg);

        EXPECT_THROW(CAN_FP::WindSensor tmp(msg, valid_id), std::out_of_range);
    };

    /** Garbage value test does not work because garbage values are within bounds
    cf.can_id = static_cast<canid_t>(CAN_FP::CanId::SAIL_WIND_DATA_FRAME_1);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);
    EXPECT_THROW(CAN_FP::WindSensor tmp(cf), std::out_of_range);
    */
}

/**
 * @brief Test CanTransceiver using a tmp file
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
