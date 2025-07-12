#include <chrono>
#include <custom_interfaces/msg/ais_ships.hpp>
#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/can_sim_to_boat_sim.hpp>
#include <custom_interfaces/msg/desired_heading.hpp>
#include <custom_interfaces/msg/generic_sensors.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <custom_interfaces/msg/wind_sensors.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "can_frame_parser.h"
#include "can_transceiver.h"
#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "net_node.h"

constexpr int  QUEUE_SIZE     = 10;  // Arbitrary number
constexpr auto TIMER_INTERVAL = std::chrono::milliseconds(500);

namespace msg = custom_interfaces::msg;
using CAN_FP::CanFrame;
using CAN_FP::CanId;

class CanTransceiverIntf : public NetNode
{
public:
    CanTransceiverIntf() : NetNode(ros_nodes::CAN_TRANSCEIVER)
    {
        this->declare_parameter("enabled", true);

        if (!this->get_parameter("enabled").as_bool()) {
            RCLCPP_INFO(this->get_logger(), "CAN Transceiver is DISABLED");
        } else {
            this->declare_parameter("mode", rclcpp::PARAMETER_STRING);

            rclcpp::Parameter mode_param = this->get_parameter("mode");
            std::string       mode       = mode_param.as_string();

            if (mode == SYSTEM_MODE::PROD) {
                RCLCPP_INFO(this->get_logger(), "Running CAN Transceiver in production mode");
                try {
                    can_trns_ = std::make_unique<CanTransceiver>();
                } catch (std::runtime_error err) {
                    RCLCPP_ERROR(this->get_logger(), "%s", err.what());
                    throw err;
                }
            } else if (mode == SYSTEM_MODE::DEV) {
                RCLCPP_INFO(this->get_logger(), "Running CAN Transceiver in development mode with CAN Sim Intf");
                try {
                    sim_intf_fd_ = mockCanFd("/tmp/CanSimIntfXXXXXX");
                    can_trns_    = std::make_unique<CanTransceiver>(sim_intf_fd_);
                } catch (std::runtime_error err) {
                    RCLCPP_ERROR(this->get_logger(), "%s", err.what());
                    throw err;
                }
            } else {
                std::string msg = "Error, invalid system mode" + mode;
                RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
                throw std::runtime_error(msg);
            }

            ais_pub_          = this->create_publisher<msg::AISShips>(ros_topics::AIS_SHIPS, QUEUE_SIZE);
            batteries_pub_    = this->create_publisher<msg::HelperBattery>(ros_topics::BATTERIES, QUEUE_SIZE);
            gps_pub_          = this->create_publisher<msg::GPS>(ros_topics::GPS, QUEUE_SIZE);
            wind_sensors_pub_ = this->create_publisher<msg::WindSensors>(ros_topics::WIND_SENSORS, QUEUE_SIZE);
            filtered_wind_sensor_pub_ =
              this->create_publisher<msg::WindSensor>(ros_topics::FILTERED_WIND_SENSOR, QUEUE_SIZE);
            generic_sensors_pub_ = this->create_publisher<msg::GenericSensors>(ros_topics::DATA_SENSORS, QUEUE_SIZE);
            rudder_pub_          = this->create_publisher<msg::HelperHeading>(ros_topics::RUDDER, QUEUE_SIZE);
            temp_sensors_pub_    = this->create_publisher<msg::TempSensors>(ros_topics::TEMP_SENSORS, QUEUE_SIZE);
            ph_sensors_pub_      = this->create_publisher<msg::PhSensors>(ros_topics::PH_SENSORS, QUEUE_SIZE);
            salinity_sensors_pub_ =
              this->create_publisher<msg::SalinitySensors>(ros_topics::SALINITY_SENSORS, QUEUE_SIZE);
            pressure_sensors_pub_ =
              this->create_publisher<msg::PressureSensors>(ros_topics::PRESSURE_SENSORS, QUEUE_SIZE);

            std::vector<std::pair<CanId, std::function<void(const CanFrame &)>>> canCbs = {
              std::make_pair(
                CanId::BMS_DATA_FRAME,
                std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishBattery(frame); })),
              std::make_pair(
                CanId::PATH_GPS_DATA_FRAME,
                std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishGPS(frame); })),
              std::make_pair(
                CanId::RUDDER_DATA_FRAME,
                std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishRudder(frame); })),
              std::make_pair(CanId::SAIL_WIND, std::function<void(const CanFrame &)>([this](const CanFrame & frame) {
                                 publishWindSensor(frame);
                             })),
              std::make_pair(
                CanId::GENERIC_SENSOR_START,
                std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishGeneric(frame); })),
              std::make_pair(CanId::SAIL_AIS, std::function<void(const CanFrame &)>([this](const CanFrame & frame) {
                                 publishAIS(frame);
                             }))};

            auto append = [&](auto && v) { canCbs.insert(canCbs.end(), v.begin(), v.end()); };

            append(getCbsForRange(CanId::TEMP_SENSOR_START, CanId::TEMP_SENSOR_END, &CanTransceiverIntf::publishTemp));
            append(getCbsForRange(CanId::PH_SENSOR_START, CanId::PH_SENSOR_END, &CanTransceiverIntf::publishPh));
            append(getCbsForRange(
              CanId::SALINITY_SENSOR_START, CanId::SALINITY_SENSOR_END, &CanTransceiverIntf::publishSalinity));
            append(getCbsForRange(
              CanId::PRESSURE_SENSOR_START, CanId::PRESSURE_SENSOR_END, &CanTransceiverIntf::publishPressure));

            can_trns_->registerCanCbs(canCbs);

            sail_cmd_sub_ = this->create_subscription<msg::SailCmd>(
              ros_topics::SAIL_CMD, QUEUE_SIZE, [this](msg::SailCmd sail_cmd_) { subSailCmdCb(sail_cmd_); });
            desired_heading_sub_ = this->create_subscription<msg::DesiredHeading>(
              ros_topics::DESIRED_HEADING, QUEUE_SIZE,
              [this](msg::DesiredHeading desired_heading_) { subDesiredHeadingCb(desired_heading_); });

            if (mode == SYSTEM_MODE::DEV) {  // Initialize the CAN Sim Intf
                mock_ais_sub_ = this->create_subscription<msg::AISShips>(
                  ros_topics::MOCK_AIS_SHIPS, QUEUE_SIZE,
                  [this](msg::AISShips mock_ais_ships) { subMockAISCb(mock_ais_ships); });
                mock_gps_sub_ = this->create_subscription<msg::GPS>(
                  ros_topics::MOCK_GPS, QUEUE_SIZE, [this](msg::GPS mock_gps) { subMockGpsCb(mock_gps); });
                mock_wind_sensors_sub_ = this->create_subscription<msg::WindSensors>(
                  ros_topics::MOCK_WIND_SENSORS, QUEUE_SIZE,
                  [this](msg::WindSensors mock_wind_sensors) { subMockWindSensorsCb(mock_wind_sensors); });
                boat_sim_input_pub_ =
                  this->create_publisher<msg::CanSimToBoatSim>(ros_topics::BOAT_SIM_INPUT, QUEUE_SIZE);

                timer_ = this->create_wall_timer(TIMER_INTERVAL, [this]() {
                    //mockBatteriesCb();
                    publishBoatSimInput(boat_sim_input_msg_);
                    // Add any other necessary looping callbacks
                });
            }
        }
    }

private:
    // pointer to the CAN Transceiver implementation
    std::unique_ptr<CanTransceiver> can_trns_;

    // Universal publishers and subscribers present in both deployment and simulation
    rclcpp::Publisher<msg::AISShips>::SharedPtr          ais_pub_;
    msg::AISShips                                        ais_ships_;
    rclcpp::Publisher<msg::HelperBattery>::SharedPtr     batteries_pub_;
    msg::HelperBattery                                   batteries_;
    rclcpp::Publisher<msg::GPS>::SharedPtr               gps_pub_;
    msg::GPS                                             gps_;
    rclcpp::Publisher<msg::WindSensors>::SharedPtr       wind_sensors_pub_;
    msg::WindSensors                                     wind_sensors_;
    rclcpp::Publisher<msg::WindSensor>::SharedPtr        filtered_wind_sensor_pub_;
    msg::WindSensor                                      filtered_wind_sensor_;
    rclcpp::Publisher<msg::GenericSensors>::SharedPtr    generic_sensors_pub_;
    msg::GenericSensors                                  generic_sensors_;
    rclcpp::Subscription<msg::DesiredHeading>::SharedPtr desired_heading_sub_;
    msg::DesiredHeading                                  desired_heading_;
    rclcpp::Subscription<msg::SailCmd>::SharedPtr        sail_cmd_sub_;
    msg::SailCmd                                         sail_cmd_;
    rclcpp::Publisher<msg::HelperHeading>::SharedPtr     rudder_pub_;
    msg::HelperHeading                                   rudder_;
    rclcpp::Publisher<msg::TempSensors>::SharedPtr       temp_sensors_pub_;
    msg::TempSensors                                     temp_sensors_;
    rclcpp::Publisher<msg::PhSensors>::SharedPtr         ph_sensors_pub_;
    msg::PhSensors                                       ph_sensors_;
    rclcpp::Publisher<msg::SalinitySensors>::SharedPtr   salinity_sensors_pub_;
    msg::SalinitySensors                                 salinity_sensors_;
    rclcpp::Publisher<msg::PressureSensors>::SharedPtr   pressure_sensors_pub_;
    msg::PressureSensors                                 pressure_sensors_;

    // Simulation only publishers and subscribers
    rclcpp::Subscription<msg::AISShips>::SharedPtr     mock_ais_sub_;
    rclcpp::Subscription<msg::GPS>::SharedPtr          mock_gps_sub_;
    rclcpp::Subscription<msg::WindSensors>::SharedPtr  mock_wind_sensors_sub_;
    rclcpp::Publisher<msg::CanSimToBoatSim>::SharedPtr boat_sim_input_pub_;
    msg::CanSimToBoatSim                               boat_sim_input_msg_;

    // Timer for anything that just needs a repeatedly written value in simulation
    rclcpp::TimerBase::SharedPtr timer_;

    // Holder for AISShips before publishing
    std::vector<msg::HelperAISShip> ais_ships_holder_;
    int                             ais_ships_num_;

    // Mock CAN file descriptor for simulation
    int sim_intf_fd_;

    // Saved power mode state
    uint8_t set_pwr_mode = CAN_FP::PwrMode::POWER_MODE_NORMAL;

    std::vector<std::pair<CAN_FP::CanId, std::function<void(const CanFrame &)>>> getCbsForRange(
      CAN_FP::CanId start, CAN_FP::CanId end, void (CanTransceiverIntf::*callback)(const CanFrame &))
    {
        using underlying = std::underlying_type_t<CAN_FP::CanId>;
        std::vector<std::pair<CAN_FP::CanId, std::function<void(const CanFrame &)>>> canCbs;

        for (underlying id = static_cast<underlying>(start); id <= static_cast<underlying>(end); ++id) {
            CAN_FP::CanId canId = static_cast<CAN_FP::CanId>(id);
            canCbs.emplace_back(canId, [this, callback](const CanFrame & frame) { (this->*callback)(frame); });
        }
        return canCbs;
    }

    /**
     * @brief Publish AIS ships
     *
     */
    void publishAIS(const CanFrame & ais_frame)
    {
        try {
            CAN_FP::AISShips ais_ship(ais_frame);

            if (ais_ships_num_ == 0) {
                ais_ships_num_ = ais_ship.getNumShips();
                ais_ships_holder_.reserve(ais_ships_num_);
            }

            ais_ships_holder_[ais_ship.getShipIndex()] = ais_ship.toRosMsg();  //maybe change to pushback later

            if (ais_ships_holder_.size() == static_cast<size_t>(ais_ships_num_)) {
                ais_ships_.ships = ais_ships_holder_;
                ais_pub_->publish(ais_ships_);
                ais_ships_holder_.clear();
                ais_ships_num_ = 0;  // reset the number of ships
            }
            RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), ais_ship.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct AISShips but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    /**
     * @brief Publish a battery_frame
     *        Intended to be registered as a callback with the CAN Transceiver instance
     *
     * @param battery_frame battery CAN frame read from the CAN bus
     */
    void publishBattery(const CanFrame & battery_frame)
    {
        try {
            CAN_FP::Battery      bat(battery_frame);
            msg::HelperBattery & bat_msg = batteries_;
            bat_msg                      = bat.toRosMsg();
            batteries_pub_->publish(batteries_);
            // Voltage < 10V means low power mode
            // If in low power mode, power mode will only change back to normal if voltage reaches >= 12V.
            if (bat_msg.voltage < 10) {  //NOLINT(readability-magic-numbers)
                set_pwr_mode = CAN_FP::PwrMode::POWER_MODE_LOW;
            } else if (bat_msg.voltage >= 12) {  //NOLINT(readability-magic-numbers)
                set_pwr_mode = CAN_FP::PwrMode::POWER_MODE_NORMAL;
            }
            CAN_FP::PwrMode power_mode(set_pwr_mode, CAN_FP::CanId::PWR_MODE);
            can_trns_->send(power_mode.toLinuxCan());

            // Get the current time as a time_point
            auto now = std::chrono::system_clock::now();

            // Convert it to a time_t object for extracting hours and minutes
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);

            std::stringstream ss;
            ss << currentTime;

            RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), bat.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct Battery but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    /**
     * @brief Publish a GPS frame
     *        Intended to be registered as a callback with the CAN Transceiver instance
     *
     * @param gps_frame gps CAN frame read from the CAN bus
     */
    void publishGPS(const CanFrame & gps_frame)
    {
        try {
            CAN_FP::GPS gps(gps_frame);

            msg::GPS gps_ = gps.toRosMsg();
            gps_pub_->publish(gps_);
            RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), gps.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct GPS but was out of range", getCurrentTimeString().c_str());
            return;
        }
    }

    /**
     * @brief Publish a wind sensor frame
     *
     * @param wind_sensor_frame
     */
    void publishWindSensor(const CanFrame & wind_sensor_frame)
    {
        try {
            CAN_FP::WindSensor wind_sensor(wind_sensor_frame);
            size_t             idx;
            for (size_t i = 0;; i++) {
                if ((wind_sensor.id_ == CAN_FP::WindSensor::WIND_SENSOR_IDS[i])) {
                    idx = i;
                    break;
                }
            }
            msg::WindSensor & wind_sensor_msg = wind_sensors_.wind_sensors[idx];
            wind_sensor_msg                   = wind_sensor.toRosMsg();
            wind_sensors_pub_->publish(wind_sensors_);
            publishFilteredWindSensor();
            RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), wind_sensor.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct Wind Sensor but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    /**
     * @brief Publish the filtered wind sensor data
     *
     */
    void publishFilteredWindSensor()
    {
        // TODO(): Currently a simple average of the two wind sensors, but we'll want something more substantial
        // with issue #271
        int32_t average_direction = 0;
        for (size_t i = 0; i < NUM_WIND_SENSORS; i++) {
            average_direction += wind_sensors_.wind_sensors[i].direction;
        }
        average_direction /= NUM_WIND_SENSORS;

        float average_speed = 0;
        for (size_t i = 0; i < NUM_WIND_SENSORS; i++) {
            average_speed += wind_sensors_.wind_sensors[i].speed.speed;
        }
        average_speed /= NUM_WIND_SENSORS;

        msg::HelperSpeed & filtered_speed = filtered_wind_sensor_.speed;
        filtered_speed.set__speed(average_speed);

        filtered_wind_sensor_.set__speed(filtered_speed);
        filtered_wind_sensor_.set__direction(static_cast<int16_t>(average_direction));

        filtered_wind_sensor_pub_->publish(filtered_wind_sensor_);
        std::stringstream ss;
        ss << "[WIND SENSOR] Speed: " << filtered_speed.speed << " Angle: " << average_direction;
        RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), ss.str().c_str());
    }

    void publishRudder(const CanFrame & rudder_frame)
    {
        try {
            CAN_FP::RudderData rudder(rudder_frame);
            msg::HelperHeading rudder_ = rudder.toRosMsg();
            rudder_pub_->publish(rudder_);
            RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), rudder.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct Rudder but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    /**
     * @brief Publish a TempSensor frame
     *        Intended to be registered as a callback with the CAN Transceiver instance
     *
     * @param temp_frame temp CAN frame read from the CAN bus
     */
    void publishTemp(const CanFrame & temp_frame)
    {
        try {
            CAN_FP::TempSensor temp_sensor(temp_frame);
            size_t             length = temp_sensors_.temp_sensors.size();
            size_t             idx    = length;
            for (size_t i = 0; i < length; i++) {
                if ((temp_sensor.id_ == CAN_FP::TempSensor::TEMP_SENSOR_IDS[i])) {
                    idx = i;
                    break;
                }
            }
            if (idx == length) {
                RCLCPP_WARN(this->get_logger(), "Unknown Temp sensor ID: 0x%X", temp_frame.can_id);
                return;
            }
            msg::TempSensor & temp_sensor_msg = temp_sensors_.temp_sensors[idx];
            temp_sensor_msg                   = temp_sensor.toRosMsg();
            temp_sensors_pub_->publish(temp_sensors_);
            RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), temp_sensor.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct Temp Sensor but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    void publishPh(const CanFrame & ph_frame)
    {
        try {
            CAN_FP::PhSensor ph_sensor(ph_frame);
            size_t           length = ph_sensors_.ph_sensors.size();
            size_t           idx    = length;
            for (size_t i = 0; i < length; i++) {
                if ((ph_sensor.id_ == CAN_FP::PhSensor::PH_SENSOR_IDS[i])) {
                    idx = i;
                    break;
                }
            }
            if (idx == length) {
                RCLCPP_WARN(this->get_logger(), "Unknown Ph sensor ID: 0x%X", ph_frame.can_id);
                return;
            }
            msg::PhSensor & ph_sensor_msg = ph_sensors_.ph_sensors[idx];
            ph_sensor_msg                 = ph_sensor.toRosMsg();
            ph_sensors_pub_->publish(ph_sensors_);
            RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), ph_sensor.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct Ph Sensor but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    void publishSalinity(const CanFrame & salinity_frame)
    {
        try {
            CAN_FP::SalinitySensor salinity_sensor(salinity_frame);
            size_t                 length = salinity_sensors_.salinity_sensors.size();
            size_t                 idx    = length;
            for (size_t i = 0; i < length; i++) {
                if ((salinity_sensor.id_ == CAN_FP::SalinitySensor::SALINITY_SENSOR_IDS[i])) {
                    idx = i;
                    break;
                }
            }
            if (idx == length) {
                RCLCPP_WARN(this->get_logger(), "Unknown salinity sensor ID: 0x%X", salinity_frame.can_id);
                return;
            }
            msg::SalinitySensor & salinity_sensor_msg = salinity_sensors_.salinity_sensors[idx];
            salinity_sensor_msg                       = salinity_sensor.toRosMsg();
            salinity_sensors_pub_->publish(salinity_sensors_);
            RCLCPP_INFO(
              this->get_logger(), "%s %s", getCurrentTimeString().c_str(), salinity_sensor.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct Salinity Sensor but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    void publishPressure(const CanFrame & pressure_frame)
    {
        try {
            CAN_FP::PressureSensor pressure_sensor(pressure_frame);
            size_t                 length = pressure_sensors_.pressure_sensors.size();
            size_t                 idx    = length;
            for (size_t i = 0; i < length; i++) {
                if ((pressure_sensor.id_ == CAN_FP::PressureSensor::PRESSURE_SENSOR_IDS[i])) {
                    idx = i;
                    break;
                }
            }
            if (idx == length) {
                RCLCPP_WARN(this->get_logger(), "Unknown pressure sensor ID: 0x%X", pressure_frame.can_id);
                return;
            }
            msg::PressureSensor & pressure_sensor_msg = pressure_sensors_.pressure_sensors[idx];
            pressure_sensor_msg                       = pressure_sensor.toRosMsg();
            pressure_sensors_pub_->publish(pressure_sensors_);
            RCLCPP_INFO(
              this->get_logger(), "%s %s", getCurrentTimeString().c_str(), pressure_sensor.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct Pressure Sensor but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    /**
     * @brief Publish a generic sensor frame
     *
     * @param generic_frame
     */
    void publishGeneric(const CanFrame & generic_frame)
    {
        //check all generic sensors in the ROS msg for the matching id
        //assumes this sensor is in the "generic_sensors_" array of sensors, however generic sensors do not have a constructor in can_frame_parser
        size_t length = generic_sensors_.generic_sensors.size();
        size_t idx    = length;
        for (size_t i = 0; i < length; i++) {
            if (generic_frame.can_id == generic_sensors_.generic_sensors[i].id) {
                idx = i;
                break;
            }
        }

        if (idx == length) {
            RCLCPP_WARN(this->get_logger(), "Unknown generic sensor ID: 0x%X", generic_frame.can_id);
            return;
        }

        uint64_t generic_data = 0;
        std::memcpy(&generic_data, generic_frame.data, sizeof(int64_t));
        msg::HelperGenericSensor & generic_sensor_msg = generic_sensors_.generic_sensors[idx];
        generic_sensor_msg.set__data(generic_data);
        generic_sensor_msg.set__id(generic_frame.can_id);

        generic_sensors_pub_->publish(generic_sensors_);
        std::stringstream ss;
        ss << "[GENERIC SENSOR] CanID: " << generic_frame.can_id << " Data: " << generic_data;
        RCLCPP_INFO(this->get_logger(), "%s %s", getCurrentTimeString().c_str(), ss.str().c_str());
    }

    /**
     * @brief Desired heading topic callback
     *
     * @param desired_heading desired_heading received from the Desired Heading topic
     */
    void subDesiredHeadingCb(msg::DesiredHeading desired_heading)
    {
        desired_heading_ = desired_heading;
        // try {
            auto desired_heading_frame = CAN_FP::DesiredHeading(desired_heading_, CanId::MAIN_HEADING);
            can_trns_->send(desired_heading_frame.toLinuxCan());
            RCLCPP_INFO(
              this->get_logger(), "%s %s", getCurrentTimeString().c_str(), desired_heading_frame.toString().c_str());
        // } catch (const std::out_of_range & e) {
        //     RCLCPP_WARN(
        //       this->get_logger(), "%s Attempted to construct DesiredHeading but was out of range",
        //       getCurrentTimeString().c_str());
        //     return;
        // }
    }

    /**
     * @brief SailCmd subscriber callback
     *
     * @param sail_cmd_
     */
    void subSailCmdCb(const msg::SailCmd & sail_cmd_input)
    {
        sail_cmd_                = sail_cmd_input;
        auto main_trim_tab_frame = CAN_FP::MainTrimTab(sail_cmd_, CanId::MAIN_TR_TAB);
        can_trns_->send(main_trim_tab_frame.toLinuxCan());
        RCLCPP_INFO(
          this->get_logger(), "%s %s", getCurrentTimeString().c_str(), main_trim_tab_frame.toString().c_str());
    }

    // SIMULATION CALLBACKS //

    /**
     * @brief Publish the boat sim input message
     *
     * @param boat_sim_input_msg
     */
    void publishBoatSimInput(const msg::CanSimToBoatSim & boat_sim_input_msg)
    {
        boat_sim_input_pub_->publish(boat_sim_input_msg);
    }

    /**
     * @brief SailCmd subscriber callback
     *
     * @param sail_cmd_
     */
    void subSimSailCmdCb(const msg::SailCmd & sail_cmd_input)
    {
        sail_cmd_ = sail_cmd_input;
        boat_sim_input_msg_.set__sail_cmd(sail_cmd_);
        try {
            CAN_FP::MainTrimTab main_trim_tab_frame(sail_cmd_input, CanId::MAIN_TR_TAB);
            can_trns_->send(main_trim_tab_frame.toLinuxCan());
            RCLCPP_INFO(
              this->get_logger(), "%s %s", getCurrentTimeString().c_str(), main_trim_tab_frame.toString().c_str());
        } catch (std::out_of_range err) {
            RCLCPP_WARN(
              this->get_logger(), "%s Attempted to construct MainTrimTab but was out of range",
              getCurrentTimeString().c_str());
            return;
        }
    }

    /**
     * @brief Mock AIS topic callback
     *
     * @param mock_ais_ships ais_ships received from the Mock AIS topic
     */
    void subMockAISCb(msg::AISShips mock_ais_ships)
    {
        ais_ships_ = mock_ais_ships;
        ais_pub_->publish(ais_ships_);
    }

    /**
     * @brief Mock GPS topic callback
     *
     * @param mock_gps mock_gps received from the Mock GPS topic
     */
    void subMockGpsCb(msg::GPS mock_gps) { gps_ = mock_gps; }

    /**
     * @brief Mock wind sensors topic callback
     *
     * @param mock_wind_sensors mock_wind_sensors received from the Mock Wind Sensors topic
     */
    void subMockWindSensorsCb(msg::WindSensors mock_wind_sensors) { wind_sensors_ = mock_wind_sensors; }

    /**
     * @brief A mock batteries callback that just sends dummy (but valid) battery values to the simulation CAN intf
     *        Intended to be continuously invoked in a loop every once in a while
     *
     */
    void mockBatteriesCb()
    {
        msg::HelperBattery bat;
        bat.set__voltage(BATT_VOLT_UBND);
        bat.set__current(BATT_CURR_UBND);

        can_trns_->send(CAN_FP::Battery(bat, CanId::BMS_DATA_FRAME).toLinuxCan());
    }
    static std::string getCurrentTimeString()
    {
        auto        now      = rclcpp::Clock().now();
        std::time_t time_now = static_cast<std::time_t>(now.seconds());
        std::tm     time_info;
        localtime_r(&time_now, &time_info);

        std::ostringstream ss;
        ss << '[' << std::put_time(&time_info, "%Y-%m-%d %H:%M:%S") << ']';

        return ss.str();
    }
};

int main(int argc, char * argv[])
{
    bool err = false;
    rclcpp::init(argc, argv);
    try {
        std::shared_ptr<CanTransceiverIntf> node = std::make_shared<CanTransceiverIntf>();
        while (rclcpp::ok()) {
            try {
                rclcpp::spin(node);
            } catch (const std::out_of_range & e) {
                RCLCPP_WARN(node->get_logger(), "%s", e.what());
            } catch (const CAN_FP::CanIdMismatchException & e) {
                RCLCPP_WARN(node->get_logger(), "%s", e.what());
            } catch (const std::exception & e) {
                RCLCPP_ERROR(node->get_logger(), "%s", e.what());
                break;
            }
        }
    } catch (std::exception & e) {
        std::cerr << e.what() << std::endl;
        err = true;
    }
    rclcpp::shutdown();
    return err ? -1 : 0;
}
