#include <errno.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>

#include "can_frame_parser.h"
#include "can_log_replayer.h"
#include "can_transceiver.h"
#include "cmn_hdrs/shared_constants.h"
#include "mock_can_bus.h"

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
 * @brief Test ROS<->CAN Battery translations work as expected for valid input values.
 *        Treat both batteries as one combined battery.
 * @brief Test ROS<->CAN Battery translations work as expected for valid input values.
 *        Treat both batteries as one combined battery.
 */
TEST_F(TestCanFrameParser, BatteryTestValid)
{
    constexpr std::array<float, NUM_BATTERIES>   expected_volts{12.5, 10.6};
    constexpr std::array<float, NUM_BATTERIES>   expected_currs{2.5, -1.0};  // negative currents are valid
    constexpr std::array<int16_t, NUM_BATTERIES> expected_raw_volts{1250, 1060};
    constexpr std::array<int16_t, NUM_BATTERIES> expected_raw_expected_currs{250, -100};

    for (size_t i = 0; i < NUM_BATTERIES; i++) {
        CAN_FP::CanId      id            = CAN_FP::CanId::BMS_DATA_FRAME;
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
    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::Battery tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<float> invalid_volts{BATT_VOLT_LBND - 1, BATT_VOLT_UBND + 1};
    std::vector<float> invalid_currs{BATT_CURR_LBND - 1, BATT_CURR_UBND + 1};

    CAN_FP::CanId      valid_id = CAN_FP::CanId::BMS_DATA_FRAME;
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

    cf.can_id = static_cast<canid_t>(CAN_FP::CanId::BMS_DATA_FRAME);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);

    EXPECT_THROW(CAN_FP::Battery tmp(cf), std::out_of_range);
}

/**
 * @brief Test ROS<->CAN MainTrimTab translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, MainTrimTabTestValid)
{
    constexpr std::array<float, 2> expected_angles{12, 33};
    CAN_FP::CanId                  id = CAN_FP::CanId::MAIN_TR_TAB;

    for (size_t i = 0; i < 2; i++) {
        float        expected_angle = expected_angles[i];
        msg::SailCmd msg;
        msg.set__trim_tab_angle_degrees(expected_angle);
        CAN_FP::MainTrimTab sail_from_ros = CAN_FP::MainTrimTab(msg, id);
        CAN_FP::CanFrame    cf            = sail_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::MainTrimTab::CAN_BYTE_DLEN_);

        uint32_t raw_angle;
        std::memcpy(&raw_angle, cf.data + CAN_FP::MainTrimTab::BYTE_OFF_ANGLE, sizeof(uint32_t));
        raw_angle = (raw_angle / 1000) - 90;  //NOLINT(readability-magic-numbers)
        EXPECT_EQ(raw_angle, expected_angle);
        CAN_FP::MainTrimTab sail_from_can = CAN_FP::MainTrimTab(cf);

        EXPECT_EQ(sail_from_can.id_, id);

        msg::SailCmd msg_from_can = sail_from_can.toRosMsg();

        EXPECT_DOUBLE_EQ(msg_from_can.trim_tab_angle_degrees, expected_angle);
    }
}

/**
 * @brief Test the behavior of the MainTrimTab class when given invalid Id values
 *
 */
TEST_F(TestCanFrameParser, TestMainTrimTabInvalid)
{
    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::MainTrimTab tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<float> invalid_angles{TRIM_LBND - 1, TRIM_UBND + 1};

    CAN_FP::CanId valid_id = CAN_FP::CanId::MAIN_TR_TAB;
    msg::SailCmd  msg;

    for (float invalid_angle : invalid_angles) {
        msg.set__trim_tab_angle_degrees(invalid_angle);

        EXPECT_THROW(CAN_FP::MainTrimTab tmp(msg, valid_id), std::out_of_range);
    };

    cf.can_id = static_cast<canid_t>(CAN_FP::CanId::MAIN_TR_TAB);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);

    EXPECT_THROW(CAN_FP::MainTrimTab tmp(cf), std::out_of_range);
}

/**
 * @brief Test ROS<->CAN DesiredHeading translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, DesiredHeadingTestValid)
{
    constexpr std::array<float, 2> expected_angles{9, 102};
    CAN_FP::CanId                  id = CAN_FP::CanId::MAIN_HEADING;

    for (size_t i = 0; i < 2; i++) {
        float              expected_angle = expected_angles[i];
        msg::HelperHeading helper_msg;
        helper_msg.set__heading(expected_angle);
        msg::DesiredHeading msg;
        msg.set__heading(helper_msg);
        CAN_FP::DesiredHeading heading_from_ros = CAN_FP::DesiredHeading(msg, id);
        CAN_FP::CanFrame       cf               = heading_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::DesiredHeading::CAN_BYTE_DLEN_);

        uint32_t raw_angle;
        std::memcpy(&raw_angle, cf.data + CAN_FP::DesiredHeading::BYTE_OFF_HEADING, sizeof(uint32_t));
        raw_angle /= 1000;  //NOLINT(readability-magic-numbers)
        EXPECT_EQ(raw_angle, expected_angle);

        CAN_FP::DesiredHeading heading_from_can = CAN_FP::DesiredHeading(cf);

        EXPECT_EQ(heading_from_can.id_, id);

        msg::DesiredHeading msg_from_can = heading_from_can.toRosMsg();

        EXPECT_DOUBLE_EQ(msg_from_can.heading.heading, expected_angle);
    }
}

/**
 * @brief Test the behavior of the DesiredHeading class when given invalid Id values
 *
 */
TEST_F(TestCanFrameParser, TestDesiredHeadingInvalid)
{
    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::DesiredHeading tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<float> invalid_angles{-361, 361};  //NOLINT(readability-magic-numbers)

    CAN_FP::CanId       valid_id = CAN_FP::CanId::MAIN_HEADING;
    msg::DesiredHeading msg;
    msg::HelperHeading  helper_msg;

    for (float invalid_angle : invalid_angles) {
        helper_msg.set__heading(invalid_angle);
        msg.set__heading(helper_msg);

        EXPECT_THROW(CAN_FP::DesiredHeading tmp(msg, valid_id), std::out_of_range);
    };

    cf.can_id = static_cast<canid_t>(CAN_FP::CanId::MAIN_HEADING);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);

    EXPECT_THROW(CAN_FP::DesiredHeading tmp(cf), std::out_of_range);
}

/**
 * @brief Test that the DesiredHeading sail flag correctly sets the steering enable bit (bit b) on the
 *        outgoing CAN frame. If sail = false, it should set the enable bit to 1 (to disable steering).
 *
 */
TEST_F(TestCanFrameParser, DesiredHeadingSailTestValid)
{
    constexpr uint8_t STEERING_ENABLE_BIT = 0b01000000;  //NOLINT(readability-magic-numbers)
    CAN_FP::CanId     id                  = CAN_FP::CanId::MAIN_HEADING;

    for (bool sail : {true, false}) {
        msg::HelperHeading helper_msg;
        helper_msg.set__heading(45);  //NOLINT(readability-magic-numbers)
        msg::DesiredHeading msg;
        msg.set__heading(helper_msg);
        msg.set__sail(sail);

        CAN_FP::DesiredHeading heading_from_ros = CAN_FP::DesiredHeading(msg, id);
        CAN_FP::CanFrame       cf               = heading_from_ros.toLinuxCan();

        uint8_t raw_steering;
        std::memcpy(&raw_steering, cf.data + CAN_FP::DesiredHeading::BYTE_OFF_STEERING, sizeof(uint8_t));

        // sail == false => enable bit set to 1 (give up steering)
        // sail == true => enable bit set to 0 (regular steering operations)
        EXPECT_EQ((raw_steering & STEERING_ENABLE_BIT) != 0, !sail);
    }
}

/**
 * @brief Test ROS<->CAN RudderData translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, RudderDataTestValid)
{
    constexpr std::array<float, 2> expected_angles{12, 128};
    CAN_FP::CanId                  id = CAN_FP::CanId::RUDDER_DATA_FRAME;

    for (size_t i = 0; i < 2; i++) {
        float              expected_angle = expected_angles[i];
        msg::HelperHeading helper_msg;
        helper_msg.set__heading(expected_angle);

        CAN_FP::RudderData heading_from_ros = CAN_FP::RudderData(helper_msg, id);
        CAN_FP::CanFrame   cf               = heading_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::RudderData::CAN_BYTE_DLEN_);

        uint32_t raw_angle;
        std::memcpy(&raw_angle, cf.data + CAN_FP::RudderData::BYTE_OFF_HEADING, sizeof(uint32_t));
        raw_angle /= 1000;  //NOLINT(readability-magic-numbers)
        EXPECT_EQ(raw_angle, expected_angle);

        CAN_FP::RudderData heading_from_can = CAN_FP::RudderData(cf);

        EXPECT_EQ(heading_from_can.id_, id);

        msg::HelperHeading msg_from_can = heading_from_can.toRosMsg();

        EXPECT_DOUBLE_EQ(msg_from_can.heading, expected_angle);
    }
}

/**
 * @brief Test the behavior of the RudderData class when given invalid Id values
 *
 */
TEST_F(TestCanFrameParser, TestRudderDataInvalid)
{
    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::RudderData tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<float> invalid_angles{-361, 361};  //NOLINT(readability-magic-numbers)

    CAN_FP::CanId      valid_id = CAN_FP::CanId::MAIN_HEADING;
    msg::HelperHeading helper_msg;

    for (float invalid_angle : invalid_angles) {
        helper_msg.set__heading(invalid_angle);

        EXPECT_THROW(CAN_FP::RudderData tmp(helper_msg, valid_id), std::out_of_range);
    };

    cf.can_id = static_cast<canid_t>(CAN_FP::CanId::RUDDER_DATA_FRAME);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);

    EXPECT_THROW(CAN_FP::RudderData tmp(cf), std::out_of_range);
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

        //expect within 0.05 knots (0.0926km/hr)
        EXPECT_NEAR(converted_speed, expected_speeds[i], 0.0926);
        EXPECT_EQ(raw_angle, expected_angles[i]);

        CAN_FP::WindSensor sensor_from_can = CAN_FP::WindSensor(cf);

        EXPECT_EQ(sensor_from_can.id_, id);

        msg::WindSensor msg_from_sensor = sensor_from_can.toRosMsg();
        //expect within 0.05 knots (0.0926km/hr)
        EXPECT_NEAR(msg_from_sensor.speed.speed, expected_speed, 0.0926);
        EXPECT_DOUBLE_EQ(msg_from_sensor.direction, expected_angle);
    }
}
/**
 * @brief Test the behavior of the WindSensor class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestWindSensorInvalid)
{
    auto optId = CAN_FP::WindSensor::rosIdxToCanId(NUM_WIND_SENSORS);
    EXPECT_FALSE(optId.has_value());

    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::WindSensor tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<int16_t> invalid_angles{-1000, 361};  //NOLINT(readability-magic-numbers)
    std::vector<float>   invalid_speeds{WIND_SPEED_LBND - 1, WIND_SPEED_UBND + 1};

    optId = CAN_FP::WindSensor::rosIdxToCanId(0);
    ASSERT_TRUE(optId.has_value());

    CAN_FP::CanId   valid_id = optId.value();
    msg::WindSensor msg;

    // Set a valid speed for this portion
    for (int16_t invalid_angle : invalid_angles) {
        msg.set__direction(invalid_angle);
        msg::HelperSpeed tmp_speed_msg;
        tmp_speed_msg.set__speed(WIND_SPEED_LBND);
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
}

/**
 * @brief Test ROS<->CAN GPS translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, GPSTestValid)
{
    constexpr std::array<float, 2>   expected_lats{48.6, -57.3};
    constexpr std::array<float, 2>   expected_lons{-32.1, 112.9};
    constexpr std::array<float, 2>   expected_speeds{3.5, 8.0};
    constexpr std::array<int32_t, 2> expected_raw_lats{138600000, 32700000};
    constexpr std::array<int32_t, 2> expected_raw_lons{147900000, 292900000};
    constexpr std::array<int32_t, 2> expected_raw_speeds{3500, 8000};

    for (size_t i = 0; i < 2; i++) {
        CAN_FP::CanId id             = CAN_FP::CanId::PATH_GPS_DATA_FRAME;
        float         expected_lat   = expected_lats[i];
        float         expected_lon   = expected_lons[i];
        float         expected_speed = expected_speeds[i];

        msg::GPS           msg;
        msg::HelperLatLon  msg_latlon;
        msg::HelperSpeed   msg_speed;
        msg::HelperHeading msg_heading;
        msg_latlon.set__latitude(static_cast<float>(expected_lat));
        msg_latlon.set__longitude(static_cast<float>(expected_lon));
        msg.set__lat_lon(msg_latlon);

        msg_speed.set__speed(expected_speed);
        msg.set__speed(msg_speed);

        msg_heading.set__heading(0.0);
        msg.set__heading(msg_heading);

        CAN_FP::GPS      gps_from_ros = CAN_FP::GPS(msg, id);
        CAN_FP::CanFrame cf           = gps_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::GPS::CAN_BYTE_DLEN_);

        int32_t raw_lat;
        int32_t raw_lon;
        int32_t raw_speed;
        std::memcpy(&raw_lat, cf.data + CAN_FP::GPS::BYTE_OFF_LAT, sizeof(int32_t));
        std::memcpy(&raw_lon, cf.data + CAN_FP::GPS::BYTE_OFF_LON, sizeof(int32_t));
        std::memcpy(&raw_speed, cf.data + CAN_FP::GPS::BYTE_OFF_SPEED, sizeof(int32_t));

        EXPECT_NEAR(raw_lat, expected_raw_lats[i], 5);
        EXPECT_NEAR(raw_lon, expected_raw_lons[i], 5);
        EXPECT_EQ(raw_speed, expected_raw_speeds[i]);

        CAN_FP::GPS gps_from_can = CAN_FP::GPS(cf);

        EXPECT_EQ(gps_from_can.id_, id);

        msg::GPS msg_from_gps = gps_from_can.toRosMsg();

        EXPECT_EQ(msg_from_gps.lat_lon.latitude, expected_lat);
        EXPECT_EQ(msg_from_gps.lat_lon.longitude, expected_lon);
        EXPECT_DOUBLE_EQ(msg_from_gps.speed.speed, expected_speed);
    }
}

/**
 * @brief Test the behavior of the GPS class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestGPSInvalid)
{
    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::GPS tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<float> invalid_lons{LON_LBND - 1, LON_UBND + 1};
    std::vector<float> invalid_lats{LAT_LBND - 1, LAT_UBND + 1};
    std::vector<float> invalid_speeds{BOAT_SPEED_LBND - 1, BOAT_SPEED_UBND + 1};

    CAN_FP::CanId valid_id = CAN_FP::CanId::PATH_GPS_DATA_FRAME;
    msg::GPS      msg;

    // Set a valid speed for this portion
    for (float invalid_lon : invalid_lons) {
        msg::HelperLatLon  msg_latlon;
        msg::HelperSpeed   msg_speed;
        msg::HelperHeading msg_heading;
        msg_latlon.set__latitude(LAT_UBND);
        msg_latlon.set__longitude(invalid_lon);
        msg.set__lat_lon(msg_latlon);

        msg_speed.set__speed(BOAT_SPEED_UBND);
        msg.set__speed(msg_speed);

        msg_heading.set__heading(HEADING_UBND);
        msg.set__heading(msg_heading);

        EXPECT_THROW(CAN_FP::GPS tmp(msg, valid_id), std::out_of_range);
    };

    // Set a valid direction for this portion
    for (float invalid_lat : invalid_lats) {
        msg::HelperLatLon  msg_latlon;
        msg::HelperSpeed   msg_speed;
        msg::HelperHeading msg_heading;
        msg_latlon.set__latitude(invalid_lat);
        msg_latlon.set__longitude(LON_UBND);
        msg.set__lat_lon(msg_latlon);

        msg_speed.set__speed(BOAT_SPEED_UBND);
        msg.set__speed(msg_speed);

        msg_heading.set__heading(HEADING_UBND);
        msg.set__heading(msg_heading);

        EXPECT_THROW(CAN_FP::GPS tmp(msg, valid_id), std::out_of_range);
    };

    for (float invalid_speed : invalid_speeds) {
        msg::HelperLatLon  msg_latlon;
        msg::HelperSpeed   msg_speed;
        msg::HelperHeading msg_heading;
        msg_latlon.set__latitude(LAT_UBND);
        msg_latlon.set__longitude(LON_UBND);
        msg.set__lat_lon(msg_latlon);

        msg_speed.set__speed(invalid_speed);
        msg.set__speed(msg_speed);

        msg_heading.set__heading(HEADING_UBND);
        msg.set__heading(msg_heading);

        EXPECT_THROW(CAN_FP::GPS tmp(msg, valid_id), std::out_of_range);
    };
}

/**
 * @brief Test NET<->CAN PwrMode translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, PwrModeTestValid)
{
    constexpr std::size_t NUM_MODES = CAN_FP::PwrMode::PWR_MODES.size();  //NOLINT(readability-magic-numbers)
    constexpr std::array<uint8_t, NUM_MODES> expected_modes{
      CAN_FP::PwrMode::POWER_MODE_LOW, CAN_FP::PwrMode::POWER_MODE_NORMAL};

    for (size_t i = 0; i < NUM_MODES; i++) {
        CAN_FP::CanId id            = CAN_FP::CanId::PWR_MODE;
        uint8_t       expected_mode = expected_modes[i];

        CAN_FP::PwrMode  pwr_mode_inst = CAN_FP::PwrMode(expected_mode, id);
        CAN_FP::CanFrame cf            = pwr_mode_inst.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::PwrMode::CAN_BYTE_DLEN_);

        uint8_t raw_mode;
        std::memcpy(&raw_mode, cf.data + CAN_FP::PwrMode::BYTE_OFF_MODE, sizeof(int8_t));

        EXPECT_EQ(raw_mode, expected_mode);
    }
}

/**
 * @brief Test the behavior of the WindSensor class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestPwrModeInvalid)
{
    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::WindSensor tmp(cf), CAN_FP::CanIdMismatchException);

    std::vector<int16_t> invalid_modes{CAN_FP::PwrMode::POWER_MODE_LOW - 1, CAN_FP::PwrMode::POWER_MODE_NORMAL + 1};

    CAN_FP::CanId valid_id = CAN_FP::CanId::PWR_MODE;

    // Set a valid speed for this portion
    for (uint8_t invalid_mode : invalid_modes) {
        EXPECT_THROW(CAN_FP::PwrMode tmp(invalid_mode, valid_id), std::out_of_range);
    };
}

/**
 * @brief Test ROS<->CAN AISShips translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, AISShipsTestValid)
{
    constexpr std::array<int32_t, 2> expected_ids{1010, 9193};
    constexpr std::array<float, 2>   expected_lats{48.6, -57.3};
    constexpr std::array<float, 2>   expected_lons{-32.1, 112.9};
    constexpr std::array<float, 2>   expected_cogs{100.4, 43.2};
    constexpr std::array<float, 2>   expected_sogs{9.26, 4.0744};
    constexpr std::array<int8_t, 2>  expected_rots{-10, 50};
    constexpr std::array<float, 2>   expected_widths{4, 65};
    constexpr std::array<float, 2>   expected_lengths{15, 360};

    constexpr std::array<uint32_t, 2> expected_raw_ids{1010, 9193};
    constexpr std::array<uint32_t, 2> expected_raw_lats{138600000, 32700000};
    constexpr std::array<uint32_t, 2> expected_raw_lons{147900000, 292900000};
    constexpr std::array<uint16_t, 2> expected_raw_cogs{1004, 432};
    constexpr std::array<uint16_t, 2> expected_raw_sogs{50, 22};
    constexpr std::array<uint8_t, 2>  expected_raw_rots{118, 178};
    constexpr std::array<uint8_t, 2>  expected_raw_widths{4, 65};
    constexpr std::array<uint16_t, 2> expected_raw_lengths{15, 360};

    for (size_t i = 0; i < 2; i++) {
        CAN_FP::CanId id              = CAN_FP::CanId::SAIL_AIS;
        int32_t       expected_id     = expected_ids[i];
        float         expected_lat    = expected_lats[i];
        float         expected_lon    = expected_lons[i];
        float         expected_cog    = expected_cogs[i];
        float         expected_sog    = expected_sogs[i];
        int8_t        expected_rot    = expected_rots[i];
        float         expected_width  = expected_widths[i];
        float         expected_length = expected_lengths[i];

        msg::HelperAISShip msg;
        msg::HelperLatLon  lat_lon;
        lat_lon.set__latitude(expected_lat);
        lat_lon.set__longitude(expected_lon);

        msg::HelperHeading cog;
        cog.set__heading(expected_cog);

        msg::HelperSpeed sog;
        //convert to km/h
        sog.set__speed(expected_sog);

        msg::HelperROT rot;
        rot.set__rot(expected_rot);

        msg::HelperDimension width;
        width.set__dimension(static_cast<float>(expected_width));  //NOLINT(readability-magic-numbers)

        msg::HelperDimension length;
        length.set__dimension(static_cast<float>(expected_length));  //NOLINT(readability-magic-numbers)

        msg.set__id(expected_id);
        msg.set__lat_lon(lat_lon);
        msg.set__cog(cog);
        msg.set__sog(sog);
        msg.set__rot(rot);
        msg.set__width(width);
        msg.set__length(length);

        CAN_FP::AISShips ais_from_ros = CAN_FP::AISShips(msg, id);
        CAN_FP::CanFrame cf           = ais_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::AISShips::CAN_BYTE_DLEN_);

        int32_t  raw_id;
        uint32_t raw_lat;
        uint32_t raw_lon;
        uint16_t raw_speed;
        uint16_t raw_course;
        uint16_t raw_heading;
        uint8_t  raw_rot;
        uint16_t raw_length;
        uint8_t  raw_width;
        uint8_t  raw_idx;
        uint8_t  raw_num_ships;

        std::memcpy(&raw_id, cf.data + CAN_FP::AISShips::BYTE_OFF_ID, sizeof(int32_t));
        std::memcpy(&raw_lat, cf.data + CAN_FP::AISShips::BYTE_OFF_LAT, sizeof(int32_t));
        std::memcpy(&raw_lon, cf.data + CAN_FP::AISShips::BYTE_OFF_LON, sizeof(int32_t));
        std::memcpy(&raw_speed, cf.data + CAN_FP::AISShips::BYTE_OFF_SPEED, sizeof(int16_t));
        std::memcpy(&raw_course, cf.data + CAN_FP::AISShips::BYTE_OFF_COURSE, sizeof(int16_t));
        std::memcpy(&raw_heading, cf.data + CAN_FP::AISShips::BYTE_OFF_HEADING, sizeof(int16_t));
        std::memcpy(&raw_rot, cf.data + CAN_FP::AISShips::BYTE_OFF_ROT, sizeof(uint8_t));
        std::memcpy(&raw_length, cf.data + CAN_FP::AISShips::BYTE_OFF_LENGTH, sizeof(int16_t));
        std::memcpy(&raw_width, cf.data + CAN_FP::AISShips::BYTE_OFF_WIDTH, sizeof(int8_t));
        std::memcpy(&raw_idx, cf.data + CAN_FP::AISShips::BYTE_OFF_IDX, sizeof(int8_t));
        std::memcpy(&raw_num_ships, cf.data + CAN_FP::AISShips::BYTE_OFF_NUM_SHIPS, sizeof(int8_t));

        EXPECT_EQ(raw_id, expected_raw_ids[i]);
        EXPECT_NEAR(raw_lat, expected_raw_lats[i], 5);
        EXPECT_NEAR(raw_lon, expected_raw_lons[i], 5);
        EXPECT_EQ(raw_speed, expected_raw_sogs[i]);
        EXPECT_EQ(raw_course, expected_raw_cogs[i]);
        EXPECT_EQ(raw_rot, expected_raw_rots[i]);
        EXPECT_EQ(raw_length, expected_raw_lengths[i]);
        EXPECT_EQ(raw_width, expected_raw_widths[i]);

        CAN_FP::AISShips ais_from_can = CAN_FP::AISShips(cf);

        EXPECT_EQ(ais_from_can.id_, id);

        msg::HelperAISShip msg_from_ais = ais_from_can.toRosMsg();

        EXPECT_EQ(msg_from_ais.id, expected_id);
        EXPECT_DOUBLE_EQ(msg_from_ais.lat_lon.latitude, expected_lat);
        EXPECT_DOUBLE_EQ(msg_from_ais.lat_lon.longitude, expected_lon);
        EXPECT_DOUBLE_EQ(msg_from_ais.cog.heading, expected_cog);
        EXPECT_DOUBLE_EQ(msg_from_ais.sog.speed, expected_sog);
        EXPECT_DOUBLE_EQ(msg_from_ais.rot.rot, expected_rot);
        EXPECT_DOUBLE_EQ(msg_from_ais.width.dimension, expected_width);
        EXPECT_DOUBLE_EQ(msg_from_ais.length.dimension, expected_length);
    }
}

/**
 * @brief Test the behavior of the AISShips class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestAISShipsInvalid)
{
    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::AISShips tmp(cf), CAN_FP::CanIdMismatchException);

    constexpr std::array<float, 2>  invalid_lats{LAT_LBND - 1, LAT_UBND + 1};
    constexpr std::array<float, 2>  invalid_lons{LON_LBND - 1, LON_UBND + 1};
    constexpr std::array<float, 2>  invalid_cogs{-361, 361};
    constexpr std::array<float, 2>  invalid_sogs{SOG_SPEED_LBND - 1, SOG_SPEED_UBND + 1};
    constexpr std::array<int8_t, 2> invalid_rots{ROT_LBND - 1, ROT_UBND + 1};
    constexpr std::array<float, 2>  invalid_widths{SHIP_DIMENSION_LBND - 2, SHIP_DIMENSION_UBND + 1};
    constexpr std::array<float, 2>  invalid_lengths{SHIP_DIMENSION_LBND - 2, SHIP_DIMENSION_UBND + 1};

    CAN_FP::CanId      valid_id = CAN_FP::CanId::SAIL_AIS;
    msg::HelperAISShip msg;

    for (float invalid_lon : invalid_lons) {
        msg::HelperLatLon lat_lon;
        lat_lon.set__latitude(LAT_UBND);
        lat_lon.set__longitude(invalid_lon);

        msg::HelperHeading cog;
        cog.set__heading(HEADING_LBND);

        msg::HelperSpeed sog;
        //convert to km/h
        sog.set__speed(SOG_SPEED_LBND);

        msg::HelperROT rot;
        rot.set__rot(ROT_UBND);

        msg::HelperDimension width;
        width.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg::HelperDimension length;
        length.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg.set__id(10);  //NOLINT(readability-magic-numbers)
        msg.set__lat_lon(lat_lon);
        msg.set__cog(cog);
        msg.set__sog(sog);
        msg.set__rot(rot);
        msg.set__width(width);
        msg.set__length(length);

        EXPECT_THROW(CAN_FP::AISShips tmp(msg, valid_id), std::out_of_range);
    };

    for (float invalid_lat : invalid_lats) {
        msg::HelperLatLon lat_lon;
        lat_lon.set__latitude(invalid_lat);
        lat_lon.set__longitude(LON_UBND);

        msg::HelperHeading cog;
        cog.set__heading(HEADING_LBND);

        msg::HelperSpeed sog;
        //convert to km/h
        sog.set__speed(SOG_SPEED_LBND);

        msg::HelperROT rot;
        rot.set__rot(ROT_UBND);

        msg::HelperDimension width;
        width.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg::HelperDimension length;
        length.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg.set__id(10);  //NOLINT(readability-magic-numbers)
        msg.set__lat_lon(lat_lon);
        msg.set__cog(cog);
        msg.set__sog(sog);
        msg.set__rot(rot);
        msg.set__width(width);
        msg.set__length(length);

        EXPECT_THROW(CAN_FP::AISShips tmp(msg, valid_id), std::out_of_range);
    };

    for (float invalid_cog : invalid_cogs) {
        msg::HelperLatLon lat_lon;
        lat_lon.set__latitude(LAT_UBND);
        lat_lon.set__longitude(LON_UBND);

        msg::HelperHeading cog;
        cog.set__heading(invalid_cog);

        msg::HelperSpeed sog;
        //convert to km/h
        sog.set__speed(SOG_SPEED_LBND);

        msg::HelperROT rot;
        rot.set__rot(ROT_UBND);

        msg::HelperDimension width;
        width.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg::HelperDimension length;
        length.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg.set__id(10);  //NOLINT(readability-magic-numbers)
        msg.set__lat_lon(lat_lon);
        msg.set__cog(cog);
        msg.set__sog(sog);
        msg.set__rot(rot);
        msg.set__width(width);
        msg.set__length(length);

        EXPECT_THROW(CAN_FP::AISShips tmp(msg, valid_id), std::out_of_range);
    };

    for (float invalid_sog : invalid_sogs) {
        msg::HelperLatLon lat_lon;
        lat_lon.set__latitude(LAT_UBND);
        lat_lon.set__longitude(LON_UBND);

        msg::HelperHeading cog;
        cog.set__heading(HEADING_LBND);

        msg::HelperSpeed sog;
        //convert to km/h
        sog.set__speed(invalid_sog);

        msg::HelperROT rot;
        rot.set__rot(ROT_UBND);

        msg::HelperDimension width;
        width.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg::HelperDimension length;
        length.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg.set__id(10);  //NOLINT(readability-magic-numbers)
        msg.set__lat_lon(lat_lon);
        msg.set__cog(cog);
        msg.set__sog(sog);
        msg.set__rot(rot);
        msg.set__width(width);
        msg.set__length(length);

        EXPECT_THROW(CAN_FP::AISShips tmp(msg, valid_id), std::out_of_range);
    };

    for (int8_t invalid_rot : invalid_rots) {
        msg::HelperLatLon lat_lon;
        lat_lon.set__latitude(LAT_UBND);
        lat_lon.set__longitude(LON_UBND);

        msg::HelperHeading cog;
        cog.set__heading(HEADING_LBND);

        msg::HelperSpeed sog;
        //convert to km/h
        sog.set__speed(SOG_SPEED_UBND);

        msg::HelperROT rot;
        rot.set__rot(invalid_rot);

        msg::HelperDimension width;
        width.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg::HelperDimension length;
        length.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg.set__id(10);  //NOLINT(readability-magic-numbers)
        msg.set__lat_lon(lat_lon);
        msg.set__cog(cog);
        msg.set__sog(sog);
        msg.set__rot(rot);
        msg.set__width(width);
        msg.set__length(length);

        EXPECT_THROW(CAN_FP::AISShips tmp(msg, valid_id), std::out_of_range);
    };

    for (float invalid_width : invalid_widths) {
        msg::HelperLatLon lat_lon;
        lat_lon.set__latitude(LAT_UBND);
        lat_lon.set__longitude(LON_UBND);

        msg::HelperHeading cog;
        cog.set__heading(HEADING_LBND);

        msg::HelperSpeed sog;
        //convert to km/h
        sog.set__speed(SOG_SPEED_UBND);

        msg::HelperROT rot;
        rot.set__rot(ROT_UBND);

        msg::HelperDimension width;
        width.set__dimension(invalid_width);

        msg::HelperDimension length;
        length.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg.set__id(10);  //NOLINT(readability-magic-numbers)
        msg.set__lat_lon(lat_lon);
        msg.set__cog(cog);
        msg.set__sog(sog);
        msg.set__rot(rot);
        msg.set__width(width);
        msg.set__length(length);

        EXPECT_THROW(CAN_FP::AISShips tmp(msg, valid_id), std::out_of_range);
    };

    for (float invalid_length : invalid_lengths) {
        msg::HelperLatLon lat_lon;
        lat_lon.set__latitude(LAT_UBND);
        lat_lon.set__longitude(LON_UBND);

        msg::HelperHeading cog;
        cog.set__heading(HEADING_LBND);

        msg::HelperSpeed sog;
        //convert to km/h
        sog.set__speed(SOG_SPEED_UBND);

        msg::HelperROT rot;
        rot.set__rot(ROT_UBND);

        msg::HelperDimension width;
        width.set__dimension(SHIP_DIMENSION_LBND);  //NOLINT(readability-magic-numbers)

        msg::HelperDimension length;
        length.set__dimension(invalid_length);

        msg.set__id(10);  //NOLINT(readability-magic-numbers)
        msg.set__lat_lon(lat_lon);
        msg.set__cog(cog);
        msg.set__sog(sog);
        msg.set__rot(rot);
        msg.set__width(width);
        msg.set__length(length);

        EXPECT_THROW(CAN_FP::AISShips tmp(msg, valid_id), std::out_of_range);
    };
}

/**
 * @brief Test ROS<->CAN Temp translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, TempSensorTestValid)
{
    constexpr std::uint8_t NUM_SENSORS = CAN_FP::TempSensor::TEMP_SENSOR_IDS.size();
    // assuming measuring ocean water, possible range ~[-2:30] = ~[271:303]
    // also include upper, lower bounds
    std::vector<float> expected_temps = {TEMP_LBND};
    for (int i = 1; i < NUM_SENSORS - 1; i++) {
        expected_temps.push_back(static_cast<float>(271 + (2.3 * i)));  // NOLINT(readability-magic-numbers)
    }
    expected_temps.push_back(TEMP_UBND);

    for (size_t i = 0; i < NUM_SENSORS; i++) {
        auto optId = CAN_FP::TempSensor::rosIdxToCanId(i);

        ASSERT_TRUE(optId.has_value());

        CAN_FP::CanId   id            = optId.value();
        float           expected_temp = expected_temps[i];
        msg::TempSensor msg;
        msg::HelperTemp temp_msg;

        temp_msg.set__temp(expected_temp);
        msg.set__temp(temp_msg);

        CAN_FP::TempSensor sensor_from_ros = CAN_FP::TempSensor(msg, id);
        CAN_FP::CanFrame   cf              = sensor_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::TempSensor::CAN_BYTE_DLEN_);

        int32_t raw_temp;
        std::memcpy(&raw_temp, cf.data + CAN_FP::TempSensor::BYTE_OFF_TEMP, sizeof(int32_t));
        float converted_temp = static_cast<float>(raw_temp / 1000.0);  //NOLINT

        const float tolerance = 0.01;
        EXPECT_NEAR(converted_temp, expected_temps[i], tolerance);

        CAN_FP::TempSensor sensor_from_can = CAN_FP::TempSensor(cf);

        EXPECT_EQ(sensor_from_can.id_, id);

        msg::TempSensor msg_from_sensor = sensor_from_can.toRosMsg();
        EXPECT_NEAR(msg_from_sensor.temp.temp, expected_temp, tolerance);
    }
}
/**
 * @brief Test the behavior of the TempSensor class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestTempSensorInvalid)
{
    auto optId = CAN_FP::TempSensor::rosIdxToCanId(NUM_TEMP_SENSORS);
    EXPECT_FALSE(optId.has_value());

    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::TempSensor tmp(cf), CAN_FP::CanIdMismatchException);

    const float        small = 0.001;
    std::vector<float> invalid_temps{TEMP_LBND - small, TEMP_UBND + small};

    optId = CAN_FP::TempSensor::rosIdxToCanId(0);
    ASSERT_TRUE(optId.has_value());

    CAN_FP::CanId   valid_id = optId.value();
    msg::TempSensor msg;

    for (float invalid_temp : invalid_temps) {
        msg::HelperTemp tmp_temp_msg;
        tmp_temp_msg.set__temp(invalid_temp);
        msg.set__temp(tmp_temp_msg);

        EXPECT_THROW(CAN_FP::TempSensor tmp(msg, valid_id), std::out_of_range);
    };
}

/**
 * @brief Test ROS<->CAN Ph translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, PhSensorTestValid)
{
    constexpr std::uint8_t NUM_SENSORS = CAN_FP::PhSensor::PH_SENSOR_IDS.size();
    std::vector<float>     expected_phs;
    float                  increment = (PH_UBND - PH_LBND) / NUM_SENSORS;
    for (int i = 0; i < NUM_SENSORS; i++) {
        expected_phs.push_back(PH_LBND + (increment * static_cast<float>(i)));
    }

    for (size_t i = 0; i < NUM_SENSORS; i++) {
        auto optId = CAN_FP::PhSensor::rosIdxToCanId(i);

        ASSERT_TRUE(optId.has_value());

        CAN_FP::CanId id          = optId.value();
        float         expected_ph = expected_phs[i];
        msg::PhSensor msg;
        msg::HelperPh ph_msg;

        ph_msg.set__ph(expected_ph);
        msg.set__ph(ph_msg);

        CAN_FP::PhSensor sensor_from_ros = CAN_FP::PhSensor(msg, id);
        CAN_FP::CanFrame cf              = sensor_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::PhSensor::CAN_BYTE_DLEN_);

        int16_t raw_ph;
        std::memcpy(&raw_ph, cf.data + CAN_FP::PhSensor::BYTE_OFF_PH, sizeof(int16_t));
        float converted_ph = static_cast<float>(raw_ph / 1000.0);  //NOLINT(readability-magic-numbers)

        const float tolerance = 0.001;
        EXPECT_NEAR(converted_ph, expected_phs[i], tolerance);

        CAN_FP::PhSensor sensor_from_can = CAN_FP::PhSensor(cf);

        EXPECT_EQ(sensor_from_can.id_, id);

        msg::PhSensor msg_from_sensor = sensor_from_can.toRosMsg();
        EXPECT_NEAR(msg_from_sensor.ph.ph, expected_ph, tolerance);
    }
}

/**
 * @brief Test the behavior of the PhSensor class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestPhSensorInvalid)
{
    auto optId = CAN_FP::PhSensor::rosIdxToCanId(NUM_PH_SENSORS);
    EXPECT_FALSE(optId.has_value());

    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::PhSensor tmp(cf), CAN_FP::CanIdMismatchException);

    const float        small = 0.001;
    std::vector<float> invalid_phs{PH_LBND - small, PH_UBND + small};

    optId = CAN_FP::PhSensor::rosIdxToCanId(0);
    ASSERT_TRUE(optId.has_value());

    CAN_FP::CanId valid_id = optId.value();
    msg::PhSensor msg;

    for (float invalid_ph : invalid_phs) {
        msg::HelperPh tmp_ph_msg;
        tmp_ph_msg.set__ph(invalid_ph);
        msg.set__ph(tmp_ph_msg);

        EXPECT_THROW(CAN_FP::PhSensor tmp(msg, valid_id), std::out_of_range);
    };
}

/**
 * @brief Test ROS<->CAN Salinity translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, SalinitySensorTestValid)
{
    constexpr std::uint8_t NUM_SENSORS = CAN_FP::SalinitySensor::SALINITY_SENSOR_IDS.size();
    std::vector<float>     expected_salinities;
    float                  increment = (SALINITY_UBND - SALINITY_LBND) / NUM_SENSORS;
    for (int i = 0; i < NUM_SENSORS; i++) {
        expected_salinities.push_back(SALINITY_LBND + (increment * static_cast<float>(i)));
    }

    for (size_t i = 0; i < NUM_SENSORS; i++) {
        auto optId = CAN_FP::SalinitySensor::rosIdxToCanId(i);

        ASSERT_TRUE(optId.has_value());

        CAN_FP::CanId       id                = optId.value();
        float               expected_salinity = expected_salinities[i];
        msg::SalinitySensor msg;
        msg::HelperSalinity salinity_msg;

        salinity_msg.set__salinity(expected_salinity);
        msg.set__salinity(salinity_msg);

        CAN_FP::SalinitySensor sensor_from_ros = CAN_FP::SalinitySensor(msg, id);
        CAN_FP::CanFrame       cf              = sensor_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::SalinitySensor::CAN_BYTE_DLEN_);

        int32_t raw_salinity;
        std::memcpy(&raw_salinity, cf.data + CAN_FP::SalinitySensor::BYTE_OFF_SALINITY, sizeof(int32_t));
        float converted_salinity = static_cast<float>(raw_salinity / 1000.0);  //NOLINT(readability-magic-numbers)

        const float tolerance = 0.001;
        EXPECT_NEAR(converted_salinity, expected_salinities[i], tolerance);

        CAN_FP::SalinitySensor sensor_from_can = CAN_FP::SalinitySensor(cf);

        EXPECT_EQ(sensor_from_can.id_, id);

        msg::SalinitySensor msg_from_sensor = sensor_from_can.toRosMsg();
        EXPECT_NEAR(msg_from_sensor.salinity.salinity, expected_salinity, tolerance);
    }
}

/**
 * @brief Test the behavior of the SalinitySensor class when given invalid input values
 *
 */
TEST_F(TestCanFrameParser, TestSalinitySensorInvalid)
{
    auto optId = CAN_FP::SalinitySensor::rosIdxToCanId(NUM_SALINITY_SENSORS);
    EXPECT_FALSE(optId.has_value());

    CAN_FP::CanId invalid_id = CAN_FP::CanId::RESERVED;

    CAN_FP::CanFrame cf{.can_id = static_cast<canid_t>(invalid_id)};

    EXPECT_THROW(CAN_FP::SalinitySensor tmp(cf), CAN_FP::CanIdMismatchException);

    //SALINITY_UBND is large enough that the resolution is only ~0.1,
    // so a smaller increment will be rounded and ignored
    const float        small   = 0.1;
    const float        smaller = 0.001;
    std::vector<float> invalid_salinities{SALINITY_LBND - smaller, SALINITY_UBND + small};

    optId = CAN_FP::SalinitySensor::rosIdxToCanId(0);
    ASSERT_TRUE(optId.has_value());

    CAN_FP::CanId       valid_id = optId.value();
    msg::SalinitySensor msg;

    for (float invalid_salinity : invalid_salinities) {
        msg::HelperSalinity tmp_salinity_msg;
        tmp_salinity_msg.set__salinity(invalid_salinity);
        msg.set__salinity(tmp_salinity_msg);

        EXPECT_THROW(CAN_FP::SalinitySensor tmp(msg, valid_id), std::out_of_range);
    };
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
 * @brief Test that callback can be properly registered and invoked on BMS_DATA_FRAME CanId
 *
 */
TEST_F(TestCanTransceiver, TestNewBatteryValid)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::BMS_DATA_FRAME, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::BMS_DATA_FRAME)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that callback can be properly registered and invoked on MAIN_HEADING CanId
 *
 */
TEST_F(TestCanTransceiver, TestNewHeadingValid)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::MAIN_HEADING, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::MAIN_HEADING)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that callback can be properly registered and invoked on MAIN_TR_TAB CanId
 *
 */
TEST_F(TestCanTransceiver, TestNewTrimTabValid)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::MAIN_TR_TAB, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::MAIN_TR_TAB)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that callback can be properly registered and invoked on SAIL_AIS CanId
 *
 */
TEST_F(TestCanTransceiver, TestNewAISValid)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::SAIL_AIS, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::SAIL_AIS)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that callback can be properly registered and invoked on SAIL_WIND CanId
 *
 */
TEST_F(TestCanTransceiver, TestNewSailWindValid)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::SAIL_WIND, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::SAIL_WIND)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that callback can be properly registered and invoked on RUDDER_DATA_FRAME CanId
 *
 */
TEST_F(TestCanTransceiver, TestNewRudderDataValid)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::RUDDER_DATA_FRAME, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::RUDDER_DATA_FRAME)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that callback can be properly registered and invoked on PATH_GPS_DATA_FRAME CanId
 *
 */
TEST_F(TestCanTransceiver, TestNewGPSValid)
{
    volatile bool is_cb_called = false;

    std::function<void(CAN_FP::CanFrame)> test_cb = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::PATH_GPS_DATA_FRAME, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::PATH_GPS_DATA_FRAME)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that callback can be properly registered and invoked on DATA_WIND CanId
 *
 */
TEST_F(TestCanTransceiver, TestNewDataWindValid)
{
    volatile bool                         is_cb_called = false;
    std::function<void(CAN_FP::CanFrame)> test_cb      = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
        is_cb_called = true;
    };
    canbus_t_->registerCanCbs({{
      std::make_pair(CAN_FP::CanId::DATA_WIND, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::DATA_WIND)};

    canbus_t_->send(dummy_frame);

    std::this_thread::sleep_for(SLEEP_TIME);

    EXPECT_TRUE(is_cb_called);
}

/**
 * @brief Test that callback can be properly registered and invoked on TEMP CanIds
 *
 */
TEST_F(TestCanTransceiver, TestNewDataTempValid)
{
    using underlying = std::underlying_type_t<CAN_FP::CanId>;
    for (underlying id = static_cast<underlying>(CAN_FP::CanId::TEMP_SENSOR_START);
         id <= static_cast<underlying>(CAN_FP::CanId::TEMP_SENSOR_END); ++id) {
        CAN_FP::CanId sensor_id = static_cast<CAN_FP::CanId>(id);
        // for (CAN_FP::CanId id = CAN_FP::CanId::TEMP_SENSOR_START; id <= CAN_FP::CanId::TEMP_SENSOR_END; id++) {
        volatile bool                         is_cb_called = false;
        std::function<void(CAN_FP::CanFrame)> test_cb      = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
            is_cb_called = true;
        };
        canbus_t_->registerCanCbs({{
          std::make_pair(sensor_id, test_cb),
        }});

        // just need a valid and matching ID for this test
        CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(sensor_id)};

        canbus_t_->send(dummy_frame);

        std::this_thread::sleep_for(SLEEP_TIME);

        EXPECT_TRUE(is_cb_called);
    }
}

/**
 * @brief Test that callback can be properly registered and invoked on PH CanIds
 *
 */
TEST_F(TestCanTransceiver, TestNewDataPhValid)
{
    using underlying = std::underlying_type_t<CAN_FP::CanId>;
    for (underlying id = static_cast<underlying>(CAN_FP::CanId::PH_SENSOR_START);
         id <= static_cast<underlying>(CAN_FP::CanId::PH_SENSOR_END); ++id) {
        CAN_FP::CanId                         sensor_id    = static_cast<CAN_FP::CanId>(id);
        volatile bool                         is_cb_called = false;
        std::function<void(CAN_FP::CanFrame)> test_cb      = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
            is_cb_called = true;
        };
        canbus_t_->registerCanCbs({{
          std::make_pair(sensor_id, test_cb),
        }});

        // just need a valid and matching ID for this test
        CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(sensor_id)};

        canbus_t_->send(dummy_frame);

        std::this_thread::sleep_for(SLEEP_TIME);

        EXPECT_TRUE(is_cb_called);
    }
}

/**
 * @brief Test that callback can be properly registered and invoked on SALINITY CanIds
 *
 */
TEST_F(TestCanTransceiver, TestNewDataSalinityValid)
{
    using underlying = std::underlying_type_t<CAN_FP::CanId>;
    for (underlying id = static_cast<underlying>(CAN_FP::CanId::SALINITY_SENSOR_START);
         id <= static_cast<underlying>(CAN_FP::CanId::SALINITY_SENSOR_END); ++id) {
        CAN_FP::CanId                         sensor_id    = static_cast<CAN_FP::CanId>(id);
        volatile bool                         is_cb_called = false;
        std::function<void(CAN_FP::CanFrame)> test_cb      = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
            is_cb_called = true;
        };
        canbus_t_->registerCanCbs({{
          std::make_pair(sensor_id, test_cb),
        }});

        // just need a valid and matching ID for this test
        CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(sensor_id)};

        canbus_t_->send(dummy_frame);

        std::this_thread::sleep_for(SLEEP_TIME);

        EXPECT_TRUE(is_cb_called);
    }
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
      std::make_pair(CAN_FP::CanId::BMS_DATA_FRAME, test_cb),
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

/**
 * @brief Test that heartbeat frames (0x130-0x134) are filtered out and never dispatched
 *
 */
TEST_F(TestCanTransceiver, TestHeartbeatFramesDropped)
{
    using underlying = std::underlying_type_t<CAN_FP::CanId>;
    for (underlying id = static_cast<underlying>(CAN_FP::CanId::HEARTBEAT_START);
         id <= static_cast<underlying>(CAN_FP::CanId::HEARTBEAT_END); ++id) {
        CAN_FP::CanId heartbeat_id = static_cast<CAN_FP::CanId>(id);

        volatile bool                         is_cb_called = false;
        std::function<void(CAN_FP::CanFrame)> test_cb      = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
            is_cb_called = true;
        };
        canbus_t_->registerCanCbs({{
          std::make_pair(heartbeat_id, test_cb),
        }});

        CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(heartbeat_id)};

        canbus_t_->send(dummy_frame);

        std::this_thread::sleep_for(SLEEP_TIME);

        EXPECT_FALSE(is_cb_called);
    }
}

/**
 * @brief Test that debug frames (0x200-0x2FF) are filtered out and never dispatched
 *
 */
TEST_F(TestCanTransceiver, TestDebugFramesDropped)
{
    using underlying = std::underlying_type_t<CAN_FP::CanId>;
    for (underlying id = static_cast<underlying>(CAN_FP::CanId::HEARTBEAT_START);
         id <= static_cast<underlying>(CAN_FP::CanId::HEARTBEAT_END); ++id) {
        CAN_FP::CanId debug_id = static_cast<CAN_FP::CanId>(id);

        volatile bool                         is_cb_called = false;
        std::function<void(CAN_FP::CanFrame)> test_cb      = [&is_cb_called](CAN_FP::CanFrame /*unused*/) {
            is_cb_called = true;
        };
        canbus_t_->registerCanCbs({{
          std::make_pair(debug_id, test_cb),
        }});

        CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(debug_id)};

        canbus_t_->send(dummy_frame);

        std::this_thread::sleep_for(SLEEP_TIME);

        EXPECT_FALSE(is_cb_called);
    }
}

/**
 * @brief Test CanLogReplayer CSV parsing, filtering, and pacing math against a candump-style log
 *
 */
class TestCanLogReplayer : public ::testing::Test
{
protected:
    // Rows taken from a real boat log: classic, CAN FD (16/24/20 byte), and zero-length frames, an invalid
    // CAN ID row (0x999) and a malformed row (both of which must be dropped), and an AIS row that the log
    // writer split across two lines mid byte ("...4D 08 0" + "7 04 B8...") which must be reassembled
    static constexpr const char * FIXTURE_CSV =
      "Timestamp,Elapsed_Time_s,CAN_Message\n"
      "2026-06-06T20:16:32.154049,5645.967,can0  041  [04]  5D 01 40 00\n"
      "2026-06-06T20:16:32.164074,5645.977,can0  204  [16]  6E 23 74 46 59 48 90 59 28 23 3B 75 AF 76 41 36\n"
      "2026-06-06T20:16:32.349830,5646.163,can0  206  [24]  EE 0B 2C 07 F9 0B 99 C6 E9 07 E9 0B FE 0B 00 00 00 00 "
      "00 00 00 00 00 00\n"
      "2026-06-06T20:16:32.350068,5646.163,can0  110  [02]  2E 1B\n"
      "2026-06-06T20:16:32.364997,5646.178,can0  132  [00]\n"
      "2026-06-06T20:16:32.365073,5646.178,can0  070  [20]  4C 36 4D 08 F4 BE 62 03 00 7D 00 00 10 03 00 00 3B 12 "
      "00 00\n"
      "2026-06-06T20:16:32.451316,5646.265,can0  999  [01]  FF\n"
      "this row is malformed\n"
      "2026-06-06T20:16:32.500000,5646.314,can0  060  [32]  00 00 00 00 AB 2C 4D 08 0\n"
      "2026-06-06T20:16:32.510000,5646.324,7 04 B8 10 00 00 10 0E FF 01 00 00 00 00 00 00 01 00 00 00 00 00 00 "
      "00\n"
      "2026-06-06T20:16:32.557519,5646.371,can0  040  [04]  5F 01 53 00\n";
    // 11 rows minus the invalid ID and malformed rows, with the split AIS row reassembled into one frame
    static constexpr size_t NUM_FIXTURE_FRAMES = 8;

    std::string csv_path_;

    TestCanLogReplayer()
    {
        std::string       tmpl = "/tmp/TestCanLogReplayerXXXXXX";
        std::vector<char> tmpl_cstr(tmpl.c_str(), tmpl.c_str() + tmpl.size() + 1);
        int               fd = mkstemp(tmpl_cstr.data());
        EXPECT_NE(fd, -1);
        csv_path_ = std::string(tmpl_cstr.data());
        std::ofstream file(csv_path_);
        file << FIXTURE_CSV;
        file.close();
        close(fd);
    }
    ~TestCanLogReplayer() override { unlink(csv_path_.c_str()); }
};

/**
 * @brief Test that a candump-style CSV log parses into the expected frames, in order, with
 *        invalid IDs and malformed rows dropped
 *
 */
TEST_F(TestCanLogReplayer, ParseCsvValid)
{
    std::vector<CAN_REPLAY::TimedFrame> frames = CAN_REPLAY::CanLogReplayer::parseCsv(csv_path_);

    ASSERT_EQ(frames.size(), NUM_FIXTURE_FRAMES);

    // Classic 4 byte frame
    EXPECT_EQ(frames[0].frame.can_id, static_cast<canid_t>(CAN_FP::CanId::DATA_WIND));
    EXPECT_EQ(frames[0].frame.len, 4);
    EXPECT_EQ(frames[0].frame.data[0], 0x5D);
    EXPECT_EQ(frames[0].frame.data[1], 0x01);
    EXPECT_EQ(frames[0].frame.data[2], 0x40);
    EXPECT_EQ(frames[0].frame.data[3], 0x00);

    // Frame times come from the ISO timestamp column (as seconds since the epoch), so check the
    // delta between the first two rows: 20:16:32.164074 - 20:16:32.154049 = 10.025 ms
    EXPECT_NEAR(frames[1].t_s - frames[0].t_s, 0.010025, 1e-4);

    // CAN FD frames with >8 bytes of data
    EXPECT_EQ(frames[1].frame.can_id, 0x204);
    EXPECT_EQ(frames[1].frame.len, 16);
    EXPECT_EQ(frames[1].frame.data[15], 0x36);
    EXPECT_EQ(frames[2].frame.can_id, 0x206);
    EXPECT_EQ(frames[2].frame.len, 24);

    // Zero-length frame
    EXPECT_EQ(frames[4].frame.can_id, static_cast<canid_t>(CAN_FP::CanId::SAIL_HEARTBEAT));
    EXPECT_EQ(frames[4].frame.len, 0);

    // 20 byte GPS frame
    EXPECT_EQ(frames[5].frame.can_id, static_cast<canid_t>(CAN_FP::CanId::PATH_GPS_DATA_FRAME));
    EXPECT_EQ(frames[5].frame.len, 20);
    EXPECT_EQ(frames[5].frame.data[0], 0x4C);
    EXPECT_EQ(frames[5].frame.data[16], 0x3B);

    // The split AIS row must be reassembled, including the byte the writer split in half ("0" + "7")
    EXPECT_EQ(frames[6].frame.can_id, static_cast<canid_t>(CAN_FP::CanId::SAIL_AIS));
    EXPECT_EQ(frames[6].frame.len, 32);
    EXPECT_EQ(frames[6].frame.data[4], 0xAB);
    EXPECT_EQ(frames[6].frame.data[8], 0x07);
    EXPECT_EQ(frames[6].frame.data[9], 0x04);
    EXPECT_EQ(frames[6].frame.data[10], 0xB8);
    EXPECT_EQ(frames[6].frame.data[31], 0x00);

    // The invalid ID (0x999) and malformed rows are dropped, so the last frame is SAIL_WIND
    EXPECT_EQ(frames[NUM_FIXTURE_FRAMES - 1].frame.can_id, static_cast<canid_t>(CAN_FP::CanId::SAIL_WIND));

    // Log timestamps must be monotonically non-decreasing
    for (size_t i = 1; i < frames.size(); i++) {
        EXPECT_GE(frames[i].t_s, frames[i - 1].t_s);
    }
}

/**
 * @brief Test that parsing a nonexistent log file throws
 *
 */
TEST_F(TestCanLogReplayer, ParseCsvMissingFile)
{
    EXPECT_THROW(CAN_REPLAY::CanLogReplayer::parseCsv("/tmp/DoesNotExist.csv"), std::runtime_error);
}

/**
 * @brief Test the allow and block ID list filtering
 *
 */
TEST_F(TestCanLogReplayer, FilterAllowBlock)
{
    std::vector<CAN_REPLAY::TimedFrame> frames = CAN_REPLAY::CanLogReplayer::parseCsv(csv_path_);

    CAN_REPLAY::ReplayConfig allow_cfg;
    allow_cfg.allow_ids = {CAN_FP::CanId::DATA_WIND};
    std::vector<CAN_REPLAY::TimedFrame> allowed = CAN_REPLAY::CanLogReplayer::filter(frames, allow_cfg);
    ASSERT_EQ(allowed.size(), 1);
    EXPECT_EQ(allowed[0].frame.can_id, static_cast<canid_t>(CAN_FP::CanId::DATA_WIND));

    CAN_REPLAY::ReplayConfig block_cfg;
    block_cfg.block_ids = {CAN_FP::CanId::SAIL_HEARTBEAT};
    std::vector<CAN_REPLAY::TimedFrame> blocked = CAN_REPLAY::CanLogReplayer::filter(frames, block_cfg);
    ASSERT_EQ(blocked.size(), NUM_FIXTURE_FRAMES - 1);
    for (const CAN_REPLAY::TimedFrame & timed_frame : blocked) {
        EXPECT_NE(timed_frame.frame.can_id, static_cast<canid_t>(CAN_FP::CanId::SAIL_HEARTBEAT));
    }
}

/**
 * @brief Test the pacing delay computation for every pacing mode
 *
 */
TEST_F(TestCanLogReplayer, DelayBefore)
{
    using CAN_REPLAY::CanLogReplayer;
    using std::chrono::milliseconds;
    using std::chrono::nanoseconds;

    // Log timestamps in seconds: three in order, one that goes backwards, then a ~10 minute recording gap
    constexpr double FIRST_T_S        = 0.0;
    constexpr double SECOND_T_S       = 0.1;
    constexpr double THIRD_T_S        = 0.15;
    constexpr double OUT_OF_ORDER_T_S = 0.10;
    constexpr double LONG_GAP_T_S     = 600.0;

    constexpr double SCALED_RATE = 2.0;  // replay at double speed
    constexpr auto   PERIOD      = milliseconds{10};

    std::vector<CAN_REPLAY::TimedFrame> frames{
      {.t_s = FIRST_T_S, .frame = {}},
      {.t_s = SECOND_T_S, .frame = {}},
      {.t_s = THIRD_T_S, .frame = {}},
      {.t_s = OUT_OF_ORDER_T_S, .frame = {}},
      {.t_s = LONG_GAP_T_S, .frame = {}}};

    CAN_REPLAY::ReplayConfig cfg;

    cfg.mode = CAN_REPLAY::PacingMode::REALTIME;
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 0, cfg), nanoseconds{0});
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 1, cfg), milliseconds{100});
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 2, cfg), milliseconds{50});
    // Out of order log timestamps must clamp to no delay rather than going negative
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 3, cfg), nanoseconds{0});
    // A ~10 minute gap in the recording must be capped at max_frame_gap instead of stalling the replay
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 4, cfg), cfg.max_frame_gap);

    cfg.mode = CAN_REPLAY::PacingMode::SCALED;
    cfg.rate = SCALED_RATE;
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 1, cfg), milliseconds{50});

    cfg.mode   = CAN_REPLAY::PacingMode::FIXED_PERIOD;
    cfg.period = PERIOD;
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 0, cfg), PERIOD);
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 1, cfg), PERIOD);

    cfg.mode = CAN_REPLAY::PacingMode::FAST;
    EXPECT_EQ(CanLogReplayer::delayBefore(frames, 1, cfg), nanoseconds{0});
}

/**
 * @brief Test a MockCanBus wired to a real CanTransceiver over the socketpair
 *
 */
class TestMockCanBus : public TestCanLogReplayer
{
protected:
    static constexpr auto POLL_INTERVAL = std::chrono::milliseconds(10);
    static constexpr int  MAX_POLLS     = 200;  // up to 2s per wait to keep CI from flaking

    /**
     * @brief Poll until pred() is true or the timeout expires
     *
     * @return whether pred() became true
     */
    static bool waitFor(const std::function<bool()> & pred)
    {
        for (int i = 0; i < MAX_POLLS; i++) {
            if (pred()) {
                return true;
            }
            std::this_thread::sleep_for(POLL_INTERVAL);
        }
        return pred();
    }
};

/**
 * @brief Test that replayed frames reach a callback registered with the CanTransceiver, with
 *        their data intact
 *
 */
TEST_F(TestMockCanBus, ReplayReachesCallback)
{
    MockCanBus     bus;
    CanTransceiver trns(bus.transceiverFd());

    std::atomic<int> wind_cb_count{0};
    std::mutex       frame_mtx;
    CAN_FP::CanFrame received_frame{};
    trns.registerCanCb(std::make_pair(
      CAN_FP::CanId::DATA_WIND,
      std::function<void(const CAN_FP::CanFrame &)>([&](const CAN_FP::CanFrame & frame) {
          std::lock_guard<std::mutex> lock(frame_mtx);
          received_frame = frame;
          wind_cb_count++;
      })));

    std::vector<CAN_REPLAY::TimedFrame> frames = CAN_REPLAY::CanLogReplayer::parseCsv(csv_path_);
    CAN_REPLAY::ReplayConfig            cfg;
    cfg.mode = CAN_REPLAY::PacingMode::FAST;
    bus.startReplay(frames, cfg);

    ASSERT_TRUE(waitFor([&bus]() { return bus.replayDone(); }));
    EXPECT_EQ(bus.numFramesSent(), frames.size());
    ASSERT_TRUE(waitFor([&wind_cb_count]() { return wind_cb_count > 0; }));

    EXPECT_EQ(wind_cb_count, 1);  // the fixture has exactly one DATA_WIND frame
    std::lock_guard<std::mutex> lock(frame_mtx);
    EXPECT_EQ(received_frame.can_id, static_cast<canid_t>(CAN_FP::CanId::DATA_WIND));
    EXPECT_EQ(received_frame.len, 4);
    EXPECT_EQ(received_frame.data[0], 0x5D);
    EXPECT_EQ(received_frame.data[1], 0x01);
    EXPECT_EQ(received_frame.data[2], 0x40);
    EXPECT_EQ(received_frame.data[3], 0x00);
}

/**
 * @brief Test that frames the boat sends through the CanTransceiver are captured by the MockCanBus
 *        instead of echoing back to the boat like the file-backed mock did
 *
 */
TEST_F(TestMockCanBus, OutboundFramesCaptured)
{
    constexpr uint8_t FRAME_LEN     = 8;
    constexpr uint8_t FRAME_PAYLOAD = 0xAB;
    // How long to keep listening for a loopback frame that must never arrive
    constexpr auto LOOPBACK_GRACE = std::chrono::milliseconds(50);

    MockCanBus     bus;
    CanTransceiver trns(bus.transceiverFd());

    volatile bool is_cb_called = false;
    trns.registerCanCb(std::make_pair(
      CAN_FP::CanId::BMS_DATA_FRAME, std::function<void(const CAN_FP::CanFrame &)>(
                                       [&is_cb_called](const CAN_FP::CanFrame & /*unused*/) { is_cb_called = true; })));

    CAN_FP::CanFrame frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::BMS_DATA_FRAME)};
    frame.len     = FRAME_LEN;
    frame.data[0] = FRAME_PAYLOAD;
    trns.send(frame);

    ASSERT_TRUE(waitFor([&bus]() { return bus.outboundFrames().size() == 1; }));

    std::vector<CAN_FP::CanFrame> outbound = bus.outboundFrames();
    EXPECT_EQ(outbound[0].can_id, static_cast<canid_t>(CAN_FP::CanId::BMS_DATA_FRAME));
    EXPECT_EQ(outbound[0].len, FRAME_LEN);
    EXPECT_EQ(outbound[0].data[0], FRAME_PAYLOAD);

    // Unlike the file-backed mock, sent frames must NOT loop back to the sender
    std::this_thread::sleep_for(LOOPBACK_GRACE);
    EXPECT_FALSE(is_cb_called);
}

/**
 * @brief Test that FIXED_PERIOD pacing spaces frames out over time instead of bursting them
 *
 */
TEST_F(TestMockCanBus, FixedPeriodPacing)
{
    constexpr auto   PERIOD     = std::chrono::milliseconds(20);
    constexpr size_t NUM_FRAMES = 4;

    MockCanBus     bus;
    CanTransceiver trns(bus.transceiverFd());

    std::atomic<size_t> cb_count{0};
    trns.registerCanCb(std::make_pair(
      CAN_FP::CanId::DATA_WIND, std::function<void(const CAN_FP::CanFrame &)>(
                                  [&cb_count](const CAN_FP::CanFrame & /*unused*/) { cb_count++; })));

    std::vector<CAN_REPLAY::TimedFrame> frames(
      NUM_FRAMES, {.t_s = 0.0, .frame = {.can_id = static_cast<canid_t>(CAN_FP::CanId::DATA_WIND), .len = 4}});
    CAN_REPLAY::ReplayConfig cfg;
    cfg.mode   = CAN_REPLAY::PacingMode::FIXED_PERIOD;
    cfg.period = PERIOD;

    auto start = std::chrono::steady_clock::now();
    bus.startReplay(frames, cfg);

    ASSERT_TRUE(waitFor([&cb_count]() { return cb_count == NUM_FRAMES; }));
    auto elapsed = std::chrono::steady_clock::now() - start;

    // One period is slept before every frame, so all frames arriving takes at least NUM_FRAMES periods
    EXPECT_GE(elapsed, NUM_FRAMES * PERIOD);
}

/**
 * @brief Test that loop mode replays the log again from the top until stopped
 *
 */
TEST_F(TestMockCanBus, LoopReplay)
{
    MockCanBus     bus;
    CanTransceiver trns(bus.transceiverFd());

    std::vector<CAN_REPLAY::TimedFrame> frames(
      2, {.t_s = 0.0, .frame = {.can_id = static_cast<canid_t>(CAN_FP::CanId::DATA_WIND), .len = 4}});
    CAN_REPLAY::ReplayConfig cfg;
    cfg.mode   = CAN_REPLAY::PacingMode::FIXED_PERIOD;
    cfg.period = std::chrono::milliseconds(1);
    cfg.loop   = true;

    bus.startReplay(frames, cfg);

    // More frames than the log holds must arrive, proving the replay wrapped around
    ASSERT_TRUE(waitFor([&bus, &frames]() { return bus.numFramesSent() > frames.size(); }));

    bus.stopReplay();
    EXPECT_FALSE(bus.replayDone());  // a looping replay is never "done", it can only be stopped
}

// The undocumented attitude frame is a CAN FD frame, so it is wider than HeadingSynthesis::SRC_MIN_DLEN
constexpr uint8_t ATTITUDE_FRAME_DLEN = 16;
// Tolerance for headings that round trip through the frame's centidegree encoding
constexpr double HEADING_TOLERANCE = 0.01;

/**
 * @brief Test synthesizing RUDDER_DATA_FRAME from the undocumented attitude frame, which is how
 *        replay publishes a heading for logs recorded before the e-compass sent 0x050
 *
 */
TEST_F(TestCanLogReplayer, SynthesizeHeadingFrames)
{
    // 143.62 deg as centidegrees at the attitude frame's heading offset
    constexpr uint16_t EXPECTED_CENTIDEG = 14362;
    constexpr float    EXPECTED_HEADING  = 143.62F;

    // Log timestamps in seconds, with the attitude frame recorded after the wind frame
    constexpr double WIND_T_S     = 1.0;
    constexpr double ATTITUDE_T_S = 2.0;

    CAN_FP::CanFrame attitude{.can_id = CAN_REPLAY::HeadingSynthesis::SRC_CAN_ID};
    attitude.len = ATTITUDE_FRAME_DLEN;
    std::memcpy(attitude.data + CAN_REPLAY::HeadingSynthesis::SRC_BYTE_OFF, &EXPECTED_CENTIDEG, sizeof(uint16_t));

    CAN_FP::CanFrame wind{.can_id = static_cast<canid_t>(CAN_FP::CanId::DATA_WIND)};
    wind.len = 4;

    std::vector<CAN_REPLAY::TimedFrame> frames{
      {.t_s = WIND_T_S, .frame = wind}, {.t_s = ATTITUDE_T_S, .frame = attitude}};
    std::vector<CAN_REPLAY::TimedFrame> out = CAN_REPLAY::CanLogReplayer::synthesizeHeadingFrames(frames);

    // the wind frame is untouched and the attitude frame is kept, with the synthetic frame after it
    ASSERT_EQ(out.size(), 3);
    EXPECT_EQ(out[0].frame.can_id, static_cast<canid_t>(CAN_FP::CanId::DATA_WIND));
    EXPECT_EQ(out[1].frame.can_id, CAN_REPLAY::HeadingSynthesis::SRC_CAN_ID);
    EXPECT_EQ(out[2].frame.can_id, static_cast<canid_t>(CAN_FP::CanId::RUDDER_DATA_FRAME));

    // the synthetic frame reuses its source's timestamp so the result stays chronological
    EXPECT_DOUBLE_EQ(out[2].t_s, ATTITUDE_T_S);
    for (size_t i = 1; i < out.size(); i++) {
        EXPECT_GE(out[i].t_s, out[i - 1].t_s);
    }

    // and it must decode through the real RudderData parser to the expected heading
    CAN_FP::RudderData rudder(out[2].frame);
    EXPECT_EQ(rudder.id_, CAN_FP::CanId::RUDDER_DATA_FRAME);
    EXPECT_NEAR(rudder.toRosMsg().heading, EXPECTED_HEADING, HEADING_TOLERANCE);
}

/**
 * @brief Test that heading synthesis skips frames it cannot decode instead of emitting bad headings
 *
 */
TEST_F(TestCanLogReplayer, SynthesizeHeadingFramesSkipsInvalid)
{
    // A frame too short to hold the heading field must not produce a synthetic frame
    CAN_FP::CanFrame short_frame{.can_id = CAN_REPLAY::HeadingSynthesis::SRC_CAN_ID};
    short_frame.len = 4;

    // Nor may an out of range value, which RudderData would reject as out of bounds
    constexpr uint16_t OUT_OF_RANGE = CAN_REPLAY::HeadingSynthesis::CENTIDEG_PER_REV;
    CAN_FP::CanFrame   bad_heading{.can_id = CAN_REPLAY::HeadingSynthesis::SRC_CAN_ID};
    bad_heading.len = ATTITUDE_FRAME_DLEN;
    std::memcpy(bad_heading.data + CAN_REPLAY::HeadingSynthesis::SRC_BYTE_OFF, &OUT_OF_RANGE, sizeof(uint16_t));

    // Log timestamps in seconds, with the malformed heading recorded after the short frame
    constexpr double SHORT_FRAME_T_S = 1.0;
    constexpr double BAD_HEADING_T_S = 2.0;

    std::vector<CAN_REPLAY::TimedFrame> frames{
      {.t_s = SHORT_FRAME_T_S, .frame = short_frame}, {.t_s = BAD_HEADING_T_S, .frame = bad_heading}};
    std::vector<CAN_REPLAY::TimedFrame> out = CAN_REPLAY::CanLogReplayer::synthesizeHeadingFrames(frames);

    EXPECT_EQ(out.size(), frames.size());
    for (const CAN_REPLAY::TimedFrame & timed_frame : out) {
        EXPECT_NE(timed_frame.frame.can_id, static_cast<canid_t>(CAN_FP::CanId::RUDDER_DATA_FRAME));
    }
}

/**
 * @brief Test that a synthesized heading frame reaches a RUDDER_DATA_FRAME callback end to end,
 *        which is what lets replay publish /rudder
 *
 */
TEST_F(TestMockCanBus, SynthesizedHeadingReachesRudderCallback)
{
    constexpr uint16_t EXPECTED_CENTIDEG = 27350;  // 273.50 deg
    constexpr float    EXPECTED_HEADING  = 273.50F;

    MockCanBus     bus;
    CanTransceiver trns(bus.transceiverFd());

    std::atomic<int> cb_count{0};
    std::mutex       frame_mtx;
    CAN_FP::CanFrame received{};
    trns.registerCanCb(std::make_pair(
      CAN_FP::CanId::RUDDER_DATA_FRAME,
      std::function<void(const CAN_FP::CanFrame &)>([&](const CAN_FP::CanFrame & frame) {
          std::lock_guard<std::mutex> lock(frame_mtx);
          received = frame;
          cb_count++;
      })));

    CAN_FP::CanFrame attitude{.can_id = CAN_REPLAY::HeadingSynthesis::SRC_CAN_ID};
    attitude.len = ATTITUDE_FRAME_DLEN;
    std::memcpy(attitude.data + CAN_REPLAY::HeadingSynthesis::SRC_BYTE_OFF, &EXPECTED_CENTIDEG, sizeof(uint16_t));

    std::vector<CAN_REPLAY::TimedFrame> frames =
      CAN_REPLAY::CanLogReplayer::synthesizeHeadingFrames({{.t_s = 0.0, .frame = attitude}});
    CAN_REPLAY::ReplayConfig cfg;
    cfg.mode = CAN_REPLAY::PacingMode::FAST;
    bus.startReplay(frames, cfg);

    ASSERT_TRUE(waitFor([&cb_count]() { return cb_count > 0; }));

    std::lock_guard<std::mutex> lock(frame_mtx);
    CAN_FP::RudderData          rudder(received);
    EXPECT_NEAR(rudder.toRosMsg().heading, EXPECTED_HEADING - HEADING_UBND, HEADING_TOLERANCE);  // bounds to 180
}
