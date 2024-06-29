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
 * @brief Test ROS<->CAN SailCmd translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, SailCmdTestValid)
{
    constexpr std::array<float, 2> expected_angles{12, 128};
    CAN_FP::CanId                  id = CAN_FP::SailCmd::SAIL_CMD_IDS[0];

    for (size_t i = 0; i < 2; i++) {
        float        expected_angle = expected_angles[i];
        msg::SailCmd msg;
        msg.set__trim_tab_angle_degrees(expected_angle);
        CAN_FP::SailCmd  sail_from_ros = CAN_FP::SailCmd(msg, id);
        CAN_FP::CanFrame cf            = sail_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::SailCmd::CAN_BYTE_DLEN_);

        uint32_t raw_angle;
        std::memcpy(&raw_angle, cf.data + CAN_FP::SailCmd::BYTE_OFF_ANGLE, sizeof(uint32_t));
        raw_angle /= 1000;  //NOLINT(readability-magic-numbers)
        EXPECT_EQ(raw_angle, expected_angle);

        CAN_FP::SailCmd sail_from_can = CAN_FP::SailCmd(cf);

        EXPECT_EQ(sail_from_can.id_, id);

        msg::SailCmd msg_from_can = sail_from_can.toRosMsg();

        EXPECT_DOUBLE_EQ(msg_from_can.trim_tab_angle_degrees, expected_angle);
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

    CAN_FP::CanId valid_id = CAN_FP::CanId::MAIN_TR_TAB;
    msg::SailCmd  msg;

    for (float invalid_angle : invalid_angles) {
        msg.set__trim_tab_angle_degrees(invalid_angle);

        EXPECT_THROW(CAN_FP::SailCmd tmp(msg, valid_id), std::out_of_range);
    };

    cf.can_id = static_cast<canid_t>(CAN_FP::CanId::MAIN_TR_TAB);
    std::copy(std::begin(GARBAGE_DATA), std::end(GARBAGE_DATA), cf.data);

    EXPECT_THROW(CAN_FP::SailCmd tmp(cf), std::out_of_range);
}

/**
 * @brief Test ROS<->CAN DesiredHeading translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, DesiredHeadingTestValid)
{
    constexpr std::array<float, 2> expected_angles{9, 102};
    CAN_FP::CanId                  id = CAN_FP::DesiredHeading::DESIRED_HEADING_IDS[0];

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

    std::vector<float> invalid_angles{HEADING_LBND - 1, HEADING_UBND + 1};

    CAN_FP::CanId       valid_id = CAN_FP::CanId::MAIN_TR_TAB;
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
 * @brief Test ROS<->CAN RudderData translations work as expected for valid input values
 *
 */
TEST_F(TestCanFrameParser, RudderDataTestValid)
{
    constexpr std::array<float, 2> expected_angles{12, 128};
    CAN_FP::CanId                  id = CAN_FP::RudderData::RUDDER_DATA_IDS[0];

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

    std::vector<float> invalid_angles{HEADING_LBND - 1, HEADING_UBND + 1};

    CAN_FP::CanId      valid_id = CAN_FP::CanId::MAIN_TR_TAB;
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
    constexpr std::array<float, 2>   expected_headings{100.4, 43.2};
    constexpr std::array<int32_t, 2> expected_raw_lats{138600, 32700};
    constexpr std::array<int32_t, 2> expected_raw_lons{147900, 292900};
    constexpr std::array<int32_t, 2> expected_raw_speeds{3500, 8000};
    constexpr std::array<int32_t, 2> expected_raw_headings{100400, 43200};

    for (size_t i = 0; i < 2; i++) {
        CAN_FP::CanId id               = CAN_FP::CanId::PATH_GPS_DATA_FRAME;
        float         expected_lat     = expected_lats[i];
        float         expected_lon     = expected_lons[i];
        float         expected_speed   = expected_speeds[i];
        float         expected_heading = expected_headings[i];

        msg::GPS           msg;
        msg::HelperLatLon  msg_latlon;
        msg::HelperSpeed   msg_speed;
        msg::HelperHeading msg_heading;
        msg_latlon.set__latitude(static_cast<float>(expected_lat));
        msg_latlon.set__longitude(static_cast<float>(expected_lon));
        msg.set__lat_lon(msg_latlon);

        msg_speed.set__speed(expected_speed);
        msg.set__speed(msg_speed);

        msg_heading.set__heading(expected_heading);
        msg.set__heading(msg_heading);

        CAN_FP::GPS      gps_from_ros = CAN_FP::GPS(msg, id);
        CAN_FP::CanFrame cf           = gps_from_ros.toLinuxCan();

        EXPECT_EQ(cf.can_id, static_cast<canid_t>(id));
        EXPECT_EQ(cf.len, CAN_FP::GPS::CAN_BYTE_DLEN_);

        int32_t raw_lat;
        int32_t raw_lon;
        int32_t raw_speed;
        int32_t raw_heading;
        std::memcpy(&raw_lat, cf.data + CAN_FP::GPS::BYTE_OFF_LAT, sizeof(int32_t));
        std::memcpy(&raw_lon, cf.data + CAN_FP::GPS::BYTE_OFF_LON, sizeof(int32_t));
        std::memcpy(&raw_speed, cf.data + CAN_FP::GPS::BYTE_OFF_SPEED, sizeof(int32_t));
        std::memcpy(&raw_heading, cf.data + CAN_FP::GPS::BYTE_OFF_HEADING, sizeof(int32_t));

        EXPECT_EQ(raw_lat, expected_raw_lats[i]);
        EXPECT_EQ(raw_lon, expected_raw_lons[i]);
        EXPECT_EQ(raw_speed, expected_raw_speeds[i]);
        EXPECT_EQ(raw_heading, expected_raw_headings[i]);

        CAN_FP::GPS gps_from_can = CAN_FP::GPS(cf);

        EXPECT_EQ(gps_from_can.id_, id);

        msg::GPS msg_from_gps = gps_from_can.toRosMsg();

        EXPECT_EQ(msg_from_gps.lat_lon.latitude, expected_lat);
        EXPECT_EQ(msg_from_gps.lat_lon.longitude, expected_lon);
        EXPECT_DOUBLE_EQ(msg_from_gps.speed.speed, expected_speed);
        EXPECT_DOUBLE_EQ(msg_from_gps.heading.heading, expected_heading);
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
    std::vector<float> invalid_speeds{SPEED_LBND - 1, SPEED_UBND + 1};
    std::vector<float> invalid_headings{HEADING_LBND - 1, HEADING_UBND + 1};

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

        msg_speed.set__speed(SPEED_UBND);
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

        msg_speed.set__speed(SPEED_UBND);
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

    for (float invalid_heading : invalid_headings) {
        msg::HelperLatLon  msg_latlon;
        msg::HelperSpeed   msg_speed;
        msg::HelperHeading msg_heading;
        msg_latlon.set__latitude(LAT_UBND);
        msg_latlon.set__longitude(LON_UBND);
        msg.set__lat_lon(msg_latlon);

        msg_speed.set__speed(SPEED_UBND);
        msg.set__speed(msg_speed);

        msg_heading.set__heading(invalid_heading);
        msg.set__heading(msg_heading);

        EXPECT_THROW(CAN_FP::GPS tmp(msg, valid_id), std::out_of_range);
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
    constexpr std::array<uint32_t, 2> expected_raw_lats{138600, 32700};
    constexpr std::array<uint32_t, 2> expected_raw_lons{147900, 292900};
    constexpr std::array<uint16_t, 2> expected_raw_cogs{1004, 432};
    constexpr std::array<uint16_t, 2> expected_raw_sogs{50, 22};
    constexpr std::array<int8_t, 2>   expected_raw_rots{-10, 50};
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
        int8_t   raw_rot;
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
        std::memcpy(&raw_rot, cf.data + CAN_FP::AISShips::BYTE_OFF_ROT, sizeof(int8_t));
        std::memcpy(&raw_length, cf.data + CAN_FP::AISShips::BYTE_OFF_LENGTH, sizeof(int16_t));
        std::memcpy(&raw_width, cf.data + CAN_FP::AISShips::BYTE_OFF_WIDTH, sizeof(int8_t));
        std::memcpy(&raw_idx, cf.data + CAN_FP::AISShips::BYTE_OFF_IDX, sizeof(int8_t));
        std::memcpy(&raw_num_ships, cf.data + CAN_FP::AISShips::BYTE_OFF_NUM_SHIPS, sizeof(int8_t));

        EXPECT_EQ(raw_id, expected_raw_ids[i]);
        EXPECT_EQ(raw_lat, expected_raw_lats[i]);
        EXPECT_EQ(raw_lon, expected_raw_lons[i]);
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
    constexpr std::array<float, 2>  invalid_cogs{HEADING_LBND - 1, HEADING_UBND + 1};
    constexpr std::array<float, 2>  invalid_sogs{SPEED_LBND - 1, SPEED_UBND + 1};
    constexpr std::array<int8_t, 2> invalid_rots{ROT_LBND - 1, ROT_UBND + 1};
    constexpr std::array<float, 2>  invalid_widths{SHIP_DIMENSION_LBND - 1, SHIP_DIMENSION_UBND + 1};
    constexpr std::array<float, 2>  invalid_lengths{SHIP_DIMENSION_LBND - 1, SHIP_DIMENSION_UBND + 1};

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
        sog.set__speed(SPEED_LBND);

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
        sog.set__speed(SPEED_LBND);

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
        sog.set__speed(SPEED_LBND);

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
        sog.set__speed(SPEED_UBND);

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
        sog.set__speed(SPEED_UBND);

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
        sog.set__speed(SPEED_UBND);

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
      std::make_pair(CAN_FP::CanId::BMS_DATA_FRAME, test_cb),
    }});

    // just need a valid and matching ID for this test
    CAN_FP::CanFrame dummy_frame{.can_id = static_cast<canid_t>(CAN_FP::CanId::BMS_DATA_FRAME)};

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
