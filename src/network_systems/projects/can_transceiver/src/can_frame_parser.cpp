#include "can_frame_parser.h"

#include <linux/can.h>

#include <cstring>
#include <iostream>
#include <span>
#include <stdexcept>

#include "cmn_hdrs/shared_constants.h"
#include "utils/utils.h"

namespace CAN_FP
{

namespace
{
/**
 * @brief Convert a CanId to its string representation
 *
 * @param id CanId to convert
 * @return std::string
 */
std::string CanIdToStr(CanId id) { return std::to_string(static_cast<canid_t>(id)); }

/**
 * @brief Get a debug string for a CanId
 *
 * @param id CanId to target
 * @return std::string
 */
std::string CanDebugStr(CanId id) { return CanIdToStr(id) + ": " + CAN_DESC.at(id); }
}  // namespace

// CanIdMismatchException START

CanIdMismatchException::CanIdMismatchException(std::span<const CanId> valid_ids, CanId received)
{
    std::string build_msg = "Mismatch between received ID: (" + CanIdToStr(received) + ") and valid IDs: \n";
    for (const CanId & id : valid_ids) {
        build_msg += CanDebugStr(id) + "\n";
    }
    msg_ = build_msg;
}

const char * CanIdMismatchException::what() { return msg_.c_str(); }

// CanIdMismatchException END
// BaseFrame START
// BaseFrame public START

std::ostream & operator<<(std::ostream & os, const BaseFrame & can) { return os << can.debugStr(); }

// BaseFrame public END
// BaseFrame protected START

BaseFrame::BaseFrame(CanId id, uint8_t can_byte_dlen) : id_(id), can_byte_dlen_(can_byte_dlen) {}

BaseFrame::BaseFrame(std::span<const CanId> valid_ids, CanId id, uint8_t can_byte_dlen) : BaseFrame(id, can_byte_dlen)
{
    bool valid = std::any_of(valid_ids.begin(), valid_ids.end(), [&id](CanId valid_id) { return id == valid_id; });
    if (!valid) {
        throw CanIdMismatchException(valid_ids, id);
    }
}

std::string BaseFrame::debugStr() const { return CanDebugStr(id_); }

CanFrame BaseFrame::toLinuxCan() const { return CanFrame{.can_id = static_cast<canid_t>(id_), .len = can_byte_dlen_}; }

// BaseFrame protected END
// BaseFrame END
// Battery START
// Battery public START

Battery::Battery(const CanFrame & cf) : Battery(static_cast<CanId>(cf.can_id))
{
    int32_t raw_volt;
    int32_t raw_curr;

    std::memcpy(&raw_volt, cf.data + BYTE_OFF_VOLT, sizeof(int32_t));
    std::memcpy(&raw_curr, cf.data + BYTE_OFF_CURR, sizeof(int32_t));

    volt_ = static_cast<float>(raw_volt) / 100;  // NOLINT(readability-magic-numbers)
    curr_ = static_cast<float>(raw_curr) / 100;  // NOLINT(readability-magic-numbers)

    checkBounds();
}

Battery::Battery(msg::HelperBattery ros_bat, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), volt_(ros_bat.voltage), curr_(ros_bat.current)
{
    checkBounds();
}

msg::HelperBattery Battery::toRosMsg() const
{
    msg::HelperBattery msg;
    msg.set__voltage(volt_);
    msg.set__current(curr_);

    return msg;
}

CanFrame Battery::toLinuxCan() const
{
    int32_t raw_volt = static_cast<int32_t>(volt_ * 100);  // NOLINT(readability-magic-numbers)
    int32_t raw_curr = static_cast<int32_t>(curr_ * 100);  // NOLINT(readability-magic-numbers)

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_VOLT, &raw_volt, sizeof(int32_t));
    std::memcpy(cf.data + BYTE_OFF_CURR, &raw_curr, sizeof(int32_t));

    return cf;
}

std::string Battery::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Voltage (V): " << volt_ << "\n"
       << "Current (A): " << curr_ << "\n";
    return ss.str();
}

// Battery public END
// Battery private START

Battery::Battery(CanId id) : BaseFrame(std::span{BATTERY_IDS}, id, CAN_BYTE_DLEN_) {}

void Battery::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(volt_, BATT_VOLT_LBND, BATT_VOLT_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Battery voltage is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(curr_, BATT_CURR_LBND, BATT_CURR_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Battery current is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

// Battery private END
// Battery END

// SailCmd START
// SailCmd public START

SailCmd::SailCmd(const CanFrame & cf) : SailCmd(static_cast<CanId>(cf.can_id))
{
    uint32_t raw_angle;

    std::memcpy(&raw_angle, cf.data + BYTE_OFF_ANGLE, sizeof(uint32_t));

    angle_ = static_cast<float>(raw_angle) / 1000;  //NOLINT(readability-magic-numbers)

    checkBounds();
}

SailCmd::SailCmd(msg::SailCmd ros_sail_cmd, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), angle_(ros_sail_cmd.trim_tab_angle_degrees)
{
    checkBounds();
}

msg::SailCmd SailCmd::toRosMsg() const
{
    msg::SailCmd msg;
    msg.set__trim_tab_angle_degrees(angle_);
    return msg;
}

CanFrame SailCmd::toLinuxCan() const
{
    uint32_t raw_angle = static_cast<uint32_t>(angle_) * 1000;  //NOLINT(readability-magic-numbers)

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_ANGLE, &raw_angle, sizeof(uint32_t));

    return cf;
}

std::string SailCmd::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Trim tab angle (degrees): " << angle_;
    return ss.str();
}

// SailCmd public END
// SailCmd private START

SailCmd::SailCmd(CanId id) : BaseFrame(std::span{SAIL_CMD_IDS}, id, CAN_BYTE_DLEN_) {}

void SailCmd::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(angle_, HEADING_LBND, HEADING_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Sail angle is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

// SailCmd private END
// SailCmd END

// WindSensor START
// WindSensor public START

WindSensor::WindSensor(const CanFrame & cf) : WindSensor(static_cast<CanId>(cf.can_id))
{
    int16_t raw_wind_speed;
    int16_t raw_wind_dir;

    std::memcpy(&raw_wind_speed, cf.data + BYTE_OFF_SPEED, sizeof(int16_t));
    std::memcpy(&raw_wind_dir, cf.data + BYTE_OFF_ANGLE, sizeof(int16_t));

    // convert knots to kmph before setting value
    wind_speed_ = static_cast<float>(raw_wind_speed * 1.852 / 10.0);  // NOLINT(readability-magic-numbers)
    wind_angle_ = static_cast<int16_t>(raw_wind_dir);

    checkBounds();
}

WindSensor::WindSensor(msg::WindSensor ros_wind_sensor, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), wind_angle_(ros_wind_sensor.direction), wind_speed_(ros_wind_sensor.speed.speed)
{
    checkBounds();
}

msg::WindSensor WindSensor::toRosMsg() const
{
    msg::WindSensor  msg;
    msg::HelperSpeed speed;
    msg.set__direction(static_cast<int16_t>(wind_angle_));
    speed.set__speed(wind_speed_);
    msg.set__speed(speed);
    return msg;
}

CanFrame WindSensor::toLinuxCan() const
{
    // convert kmph to knots before setting value
    int16_t raw_wind_speed = static_cast<int16_t>(wind_speed_ * 10 / 1.852);  // NOLINT(readability-magic-numbers)
    int16_t raw_wind_dir   = static_cast<int16_t>(wind_angle_);

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_SPEED, &raw_wind_speed, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_ANGLE, &raw_wind_dir, sizeof(int16_t));

    return cf;
}

std::string WindSensor::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Wind speed (m/s): " << wind_speed_ << "\n"
       << "Wind angle (degrees): " << wind_angle_;
    return ss.str();
}

std::optional<CanId> WindSensor::rosIdxToCanId(size_t wind_idx)
{
    if (wind_idx < WIND_SENSOR_IDS.size()) {
        return WIND_SENSOR_IDS[wind_idx];
    }
    return std::nullopt;
}
// WindSensor public END
// WindSensor private START

WindSensor::WindSensor(CanId id) : BaseFrame(std::span{WIND_SENSOR_IDS}, id, CAN_BYTE_DLEN_) {}

void WindSensor::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(wind_angle_, WIND_DIRECTION_LBND, WIND_DIRECTION_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Wind angle is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(wind_speed_, SPEED_LBND, SPEED_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Wind speed is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

// WindSensor private END
// WindSensor END

// GPS START
// GPS public START
GPS::GPS(const CanFrame & cf) : GPS(static_cast<CanId>(cf.can_id))
{
    int32_t raw_lat;
    int32_t raw_lon;
    int32_t raw_sec;
    int8_t  raw_min;
    int8_t  raw_hour;
    int32_t raw_heading;
    int32_t raw_speed;

    std::memcpy(&raw_lat, cf.data + BYTE_OFF_LAT, sizeof(int32_t));
    std::memcpy(&raw_lon, cf.data + BYTE_OFF_LON, sizeof(int32_t));
    std::memcpy(&raw_sec, cf.data + BYTE_OFF_SEC, sizeof(int32_t));
    std::memcpy(&raw_min, cf.data + BYTE_OFF_MIN, sizeof(int8_t));
    std::memcpy(&raw_hour, cf.data + BYTE_OFF_HOUR, sizeof(int8_t));
    std::memcpy(&raw_heading, cf.data + BYTE_OFF_HEADING, sizeof(int32_t));
    std::memcpy(&raw_speed, cf.data + BYTE_OFF_SPEED, sizeof(int32_t));

    lat_     = static_cast<float>(raw_lat / 1000.0 - 90);     //NOLINT(readability-magic-numbers)
    lon_     = static_cast<float>(raw_lon / 1000.0 - 180.0);  //NOLINT(readability-magic-numbers)
    sec_     = static_cast<float>(raw_sec / 1000.0);          //NOLINT(readability-magic-numbers)
    min_     = static_cast<float>(raw_min);
    hour_    = static_cast<float>(raw_hour);
    heading_ = static_cast<float>(raw_heading / 1000.0);  //NOLINT(readability-magic-numbers)
    speed_   = static_cast<float>(raw_speed / 1000.0);    //NOLINT(readability-magic-numbers)

    checkBounds();
}

GPS::GPS(msg::GPS ros_gps, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_),
  lat_(ros_gps.lat_lon.latitude),
  lon_(ros_gps.lat_lon.longitude),
  sec_(0),   // unused set to 0
  min_(0),   // unused set to 0
  hour_(0),  // unused set to 0
  heading_(ros_gps.heading.heading),
  speed_(ros_gps.speed.speed)
{
    checkBounds();
}

msg::GPS GPS::toRosMsg() const
{
    msg::GPS           msg;
    msg::HelperLatLon  lat_lon;
    msg::HelperHeading heading;
    msg::HelperSpeed   speed;
    lat_lon.set__latitude(lat_);
    lat_lon.set__longitude(lon_);
    heading.set__heading(heading_);
    speed.set__speed(speed_);
    msg.set__lat_lon(lat_lon);
    msg.set__heading(heading);
    msg.set__speed(speed);
    return msg;
}

CanFrame GPS::toLinuxCan() const
{
    int32_t raw_lat  = static_cast<int32_t>(std::round((lat_ + 90.0) * 1000.0));   //NOLINT(readability-magic-numbers)
    int32_t raw_lon  = static_cast<int32_t>(std::round((lon_ + 180.0) * 1000.0));  //NOLINT(readability-magic-numbers)
    int32_t raw_sec  = static_cast<int32_t>(sec_ * 1000);                          //NOLINT(readability-magic-numbers)
    int8_t  raw_min  = static_cast<int8_t>(min_);
    int8_t  raw_hour = static_cast<int8_t>(hour_);
    int32_t raw_heading = static_cast<int32_t>(heading_ * 1000);  //NOLINT(readability-magic-numbers)
    int32_t raw_speed   = static_cast<int32_t>(speed_ * 1000);    //NOLINT(readability-magic-numbers)

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_LAT, &raw_lat, sizeof(int32_t));
    std::memcpy(cf.data + BYTE_OFF_LON, &raw_lon, sizeof(int32_t));
    std::memcpy(cf.data + BYTE_OFF_SEC, &raw_sec, sizeof(int32_t));
    std::memcpy(cf.data + BYTE_OFF_MIN, &raw_min, sizeof(int8_t));
    std::memcpy(cf.data + BYTE_OFF_HOUR, &raw_hour, sizeof(int8_t));
    std::memcpy(cf.data + BYTE_OFF_HEADING, &raw_heading, sizeof(int32_t));
    std::memcpy(cf.data + BYTE_OFF_SPEED, &raw_speed, sizeof(int32_t));

    return cf;
}

std::string GPS::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Latitude (decimal degrees): " << lat_ << "\n"
       << "Longitude (decimal degrees): " << lon_ << "\n"
       << "Seconds (sec): " << sec_ << "\n"
       << "Minutes (min): " << min_ << "\n"
       << "Hours (hr): " << hour_ << "\n"
       << "True heading (degrees): " << heading_ << "\n"
       << "Speed (km/hr): " << speed_ << "\n";
    return ss.str();
}

//GPS public END
//GPS private START

GPS::GPS(CanId id) : BaseFrame(std::span{GPS_IDS}, id, CAN_BYTE_DLEN_) {}

void GPS::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(lat_, LAT_LBND, LAT_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Latitude angle is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(lon_, LON_LBND, LON_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Longitude is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(heading_, HEADING_LBND, HEADING_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Heading is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(speed_, SPEED_LBND, SPEED_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Speed is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

//GPS private END
//GPS END

//AISShips START
//AISShips pubic START

AISShips::AISShips(const CanFrame & cf) : AISShips(static_cast<CanId>(cf.can_id))
{
    int32_t  raw_id;  // CAN documentation says id is uint32, but custom interfaces says its int32
    uint32_t raw_lat;
    uint32_t raw_lon;
    uint16_t raw_speed;
    uint16_t raw_course;
    uint16_t raw_heading;
    int8_t   raw_rot;
    uint16_t raw_length;
    uint16_t raw_width;
    uint8_t  raw_idx;
    uint8_t  raw_num_ships;

    std::memcpy(&raw_id, cf.data + BYTE_OFF_ID, sizeof(int32_t));
    std::memcpy(&raw_lat, cf.data + BYTE_OFF_LAT, sizeof(int32_t));
    std::memcpy(&raw_lon, cf.data + BYTE_OFF_LON, sizeof(int32_t));
    std::memcpy(&raw_speed, cf.data + BYTE_OFF_SPEED, sizeof(int16_t));
    std::memcpy(&raw_course, cf.data + BYTE_OFF_COURSE, sizeof(int16_t));
    std::memcpy(&raw_heading, cf.data + BYTE_OFF_HEADING, sizeof(int16_t));
    std::memcpy(&raw_rot, cf.data + BYTE_OFF_ROT, sizeof(int8_t));
    std::memcpy(&raw_length, cf.data + BYTE_OFF_LENGTH, sizeof(int16_t));
    std::memcpy(&raw_width, cf.data + BYTE_OFF_WIDTH, sizeof(uint16_t));
    std::memcpy(&raw_idx, cf.data + BYTE_OFF_IDX, sizeof(int8_t));
    std::memcpy(&raw_num_ships, cf.data + BYTE_OFF_NUM_SHIPS, sizeof(int8_t));

    num_ships_ = raw_num_ships;
    lat_       = static_cast<float>(raw_lat / 1000.0 - 90);     //NOLINT(readability-magic-numbers)
    lon_       = static_cast<float>(raw_lon / 1000.0 - 180.0);  //NOLINT(readability-magic-numbers)
    speed_     = static_cast<float>(raw_speed / 10.0 * 1.852);  //NOLINT(readability-magic-numbers)
    rot_       = raw_rot;
    course_    = static_cast<float>(raw_course / 10.0);  //NOLINT(readability-magic-numbers)
    heading_   = raw_heading;
    idx_       = raw_idx;
    width_     = raw_width;
    length_    = raw_length;
    ship_id_   = raw_id;

    checkBounds();
}

AISShips::AISShips(msg::HelperAISShip ros_ship, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_),
  num_ships_(0),
  lat_(ros_ship.lat_lon.latitude),
  lon_(ros_ship.lat_lon.longitude),
  speed_(ros_ship.sog.speed),
  rot_(ros_ship.rot.rot),
  course_(ros_ship.cog.heading),
  width_(ros_ship.width.dimension),
  length_(ros_ship.length.dimension),
  ship_id_(ros_ship.id)
{
    checkBounds();
}

msg::HelperAISShip AISShips::toRosMsg() const
{
    msg::HelperAISShip ship;

    msg::HelperLatLon lat_lon;
    lat_lon.set__latitude(lat_);
    lat_lon.set__longitude(lon_);

    msg::HelperHeading cog;
    cog.set__heading(course_);

    msg::HelperSpeed sog;
    //convert to km/h
    sog.set__speed(speed_);

    msg::HelperROT rot;
    rot.set__rot(rot_);

    msg::HelperDimension width;
    width.set__dimension(static_cast<float>(width_));  //NOLINT(readability-magic-numbers)

    msg::HelperDimension length;
    length.set__dimension(static_cast<float>(length_));  //NOLINT(readability-magic-numbers)

    ship.set__id(ship_id_);
    ship.set__lat_lon(lat_lon);
    ship.set__cog(cog);
    ship.set__sog(sog);
    ship.set__rot(rot);
    ship.set__width(width);
    ship.set__length(length);

    return ship;
}

CanFrame AISShips::toLinuxCan() const
{
    //NOTE: Implemented rounding for km/hr to knots conversion, is this ok?
    uint32_t raw_id    = ship_id_;
    uint32_t raw_lat   = static_cast<int32_t>(std::round((lat_ + 90.0) * 1000.0));   //NOLINT(readability-magic-numbers)
    uint32_t raw_lon   = static_cast<int32_t>(std::round((lon_ + 180.0) * 1000.0));  //NOLINT(readability-magic-numbers)
    uint16_t raw_speed = static_cast<int16_t>(std::round(speed_ / 1.852 * 10));      //NOLINT(readability-magic-numbers)
    uint16_t raw_course    = static_cast<int16_t>(course_ * 10);                     //NOLINT(readability-magic-numbers)
    uint16_t raw_heading   = static_cast<int16_t>(heading_);
    int8_t   raw_rot       = rot_;
    uint16_t raw_length    = static_cast<int16_t>(length_);
    uint16_t raw_width     = static_cast<uint16_t>(width_);
    uint8_t  raw_idx       = idx_;
    uint8_t  raw_num_ships = num_ships_;

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_ID, &raw_id, sizeof(int32_t));
    std::memcpy(cf.data + BYTE_OFF_LAT, &raw_lat, sizeof(int32_t));
    std::memcpy(cf.data + BYTE_OFF_LON, &raw_lon, sizeof(int32_t));
    std::memcpy(cf.data + BYTE_OFF_SPEED, &raw_speed, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_COURSE, &raw_course, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_HEADING, &raw_heading, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_ROT, &raw_rot, sizeof(int8_t));
    std::memcpy(cf.data + BYTE_OFF_LENGTH, &raw_length, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_WIDTH, &raw_width, sizeof(uint16_t));
    std::memcpy(cf.data + BYTE_OFF_IDX, &raw_idx, sizeof(int8_t));
    std::memcpy(cf.data + BYTE_OFF_NUM_SHIPS, &raw_num_ships, sizeof(int8_t));

    return cf;
}

std::string AISShips::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Ship " << idx_ << ":\n"
       << "ID: " << ship_id_ << "\n"
       << "Latitude (decimal degrees): " << lat_ << "\n"
       << "Longitude (decimal degrees): " << lon_ << "\n"
       << "Heading (degrees): " << heading_ << "\n"
       << "Speed (km/hr): " << speed_ << "\n"
       << "ROT (degree/min): " << rot_ << "\n"
       << "Width (m): " << width_ << "\n"
       << "Length (m): " << length_ << "\n"
       << "\n";

    return ss.str();
}
//AISShips public END
//AISShips private START

AISShips::AISShips(CanId id) : BaseFrame(std::span{AISSHIPS_IDS}, id, CAN_BYTE_DLEN_) {}

float AISShips::roundFloat(float val)
{
    return std::round(val * 10000) / 10000;  //NOLINT(readability-magic-numbers)
}

void AISShips::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(lat_, LAT_LBND, LAT_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Latitude angle is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(lon_, LON_LBND, LON_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Longitude is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(speed_, SPEED_LBND, SPEED_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Speed is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(course_, HEADING_LBND, HEADING_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Course is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(rot_, ROT_LBND, ROT_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("ROT is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(width_, SHIP_DIMENSION_LBND, SHIP_DIMENSION_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Width is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}
//AISShips private END
//AISShips END

//PwRMode START
//PwRMode public START

PwrMode::PwrMode(const CanFrame & cf) : PwrMode(static_cast<CanId>(cf.can_id))
{
    uint8_t raw_mode;

    std::memcpy(&raw_mode, cf.data + BYTE_OFF_MODE, sizeof(uint8_t));

    mode_ = raw_mode;

    checkBounds();
}

PwrMode::PwrMode(uint8_t mode, CanId id) : BaseFrame(id, CAN_BYTE_DLEN_), mode_(mode) { checkBounds(); }

CanFrame PwrMode::toLinuxCan() const
{
    uint8_t raw_mode = mode_;

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_MODE, &raw_mode, sizeof(uint8_t));

    return cf;
}

std::string PwrMode::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Power mode: " << mode_;
    return ss.str();
}

// PwrMode public END
// PwrMode private START

PwrMode::PwrMode(CanId id) : BaseFrame(std::span{PWR_MODE_IDS}, id, CAN_BYTE_DLEN_) {}

void PwrMode::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(mode_, POWER_MODE_LOW, POWER_MODE_NORMAL);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Power mode value is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}
//can i add new custom int messages and can i add new constants
// PwrMode private END
// PwrMode END

// DesiredHeading START
// DesiredHeading public START

DesiredHeading::DesiredHeading(const CanFrame & cf) : DesiredHeading(static_cast<CanId>(cf.can_id))
{
    uint32_t raw_heading;

    std::memcpy(&raw_heading, cf.data + BYTE_OFF_HEADING, sizeof(uint32_t));

    heading_ = static_cast<float>(raw_heading) / 1000;  //NOLINT(readability-magic-numbers)

    checkBounds();
}

DesiredHeading::DesiredHeading(msg::DesiredHeading ros_desired_heading, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), heading_(ros_desired_heading.heading.heading)
{
    checkBounds();
}

msg::DesiredHeading DesiredHeading::toRosMsg() const
{
    msg::HelperHeading helper_msg;
    helper_msg.set__heading(heading_);
    msg::DesiredHeading msg;
    msg.set__heading(helper_msg);
    return msg;
}

CanFrame DesiredHeading::toLinuxCan() const
{
    uint32_t raw_heading = static_cast<uint32_t>(heading_) * 1000;  //NOLINT(readability-magic-numbers)

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_HEADING, &raw_heading, sizeof(uint32_t));

    return cf;
}

std::string DesiredHeading::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Desired heading: " << heading_;
    return ss.str();
}

// DesiredHeading public END
// DesiredHeading private START

DesiredHeading::DesiredHeading(CanId id) : BaseFrame(std::span{DESIRED_HEADING_IDS}, id, CAN_BYTE_DLEN_) {}

void DesiredHeading::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(heading_, HEADING_LBND, HEADING_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Desired heading is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

// DesiredHeading private END
// DesiredHeading END

// RudderData START
// RudderData public START

RudderData::RudderData(const CanFrame & cf) : RudderData(static_cast<CanId>(cf.can_id))
{
    uint32_t raw_heading;

    std::memcpy(&raw_heading, cf.data + BYTE_OFF_HEADING, sizeof(uint32_t));

    heading_ = static_cast<float>(raw_heading) / 1000;  //NOLINT(readability-magic-numbers)

    checkBounds();
}

RudderData::RudderData(msg::HelperHeading ros_rudder_data, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), heading_(ros_rudder_data.heading)
{
    checkBounds();
}

msg::HelperHeading RudderData::toRosMsg() const
{
    msg::HelperHeading msg;
    msg.set__heading(heading_);
    return msg;
}

CanFrame RudderData::toLinuxCan() const
{
    uint32_t raw_heading = static_cast<uint32_t>(heading_) * 1000;  //NOLINT(readability-magic-numbers)

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_HEADING, &raw_heading, sizeof(uint32_t));

    return cf;
}

std::string RudderData::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Rudder heading: " << heading_;
    return ss.str();
}

// DesiredHeading public END
// DesiredHeading private START

RudderData::RudderData(CanId id) : BaseFrame(std::span{RUDDER_DATA_IDS}, id, CAN_BYTE_DLEN_) {}

void RudderData::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(heading_, HEADING_LBND, HEADING_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Rudder heading is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }

    // DesiredHeading private END
    // DesiredHeading END
}
}  // namespace CAN_FP
