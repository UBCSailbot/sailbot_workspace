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
    int16_t raw_volt;
    int16_t raw_curr;
    int16_t raw_max_volt;
    int16_t raw_min_volt;

    std::memcpy(&raw_volt, cf.data + BYTE_OFF_VOLT, sizeof(int16_t));
    std::memcpy(&raw_curr, cf.data + BYTE_OFF_CURR, sizeof(int16_t));
    std::memcpy(&raw_max_volt, cf.data + BYTE_OFF_MAX_VOLT, sizeof(int16_t));
    std::memcpy(&raw_min_volt, cf.data + BYTE_OFF_MIN_VOLT, sizeof(int16_t));

    volt_ = static_cast<float>(raw_volt) / 100;  // NOLINT(readability-magic-numbers)
    curr_ = static_cast<float>(raw_curr) / 100;  // NOLINT(readability-magic-numbers)

    // TODO(hhenry01): Max and min are dodgy... it doesn't make sense to not divide them by 100 - confirm with ELEC
    volt_max_ = static_cast<float>(raw_max_volt);
    volt_min_ = static_cast<float>(raw_min_volt);

    checkBounds();
}

Battery::Battery(msg::HelperBattery ros_bat, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), volt_(ros_bat.voltage), curr_(ros_bat.current), volt_max_(0.0), volt_min_(0.0)
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
    int16_t raw_volt = static_cast<int16_t>(volt_ * 100);  // NOLINT(readability-magic-numbers)
    int16_t raw_curr = static_cast<int16_t>(curr_ * 100);  // NOLINT(readability-magic-numbers)
    // TODO(hhenry01): Max and min are dodgy... it doesn't make sense to not multiply them by 100 - confirm with ELEC
    int16_t raw_max_volt = static_cast<int16_t>(volt_max_);
    int16_t raw_min_volt = static_cast<int16_t>(volt_max_);

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_VOLT, &raw_volt, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_CURR, &raw_curr, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_MAX_VOLT, &raw_max_volt, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_MIN_VOLT, &raw_min_volt, sizeof(int16_t));

    return cf;
}

std::string Battery::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Voltage (V): " << volt_ << "\n"
       << "Current (A): " << curr_ << "\n"
       << "Max voltage (V): " << volt_max_ << "\n"
       << "Min voltage (V): " << volt_min_;
    return ss.str();
}

std::optional<CanId> Battery::rosIdxToCanId(size_t bat_idx)
{
    if (bat_idx < BATTERY_IDS.size()) {
        return BATTERY_IDS[bat_idx];
    }
    return std::nullopt;
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

}  // namespace CAN_FP
