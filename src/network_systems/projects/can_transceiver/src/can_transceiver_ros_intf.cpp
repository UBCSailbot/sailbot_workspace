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
            batteries_pub_    = this->create_publisher<msg::Batteries>(ros_topics::BATTERIES, QUEUE_SIZE);
            gps_pub_          = this->create_publisher<msg::GPS>(ros_topics::GPS, QUEUE_SIZE);
            wind_sensors_pub_ = this->create_publisher<msg::WindSensors>(ros_topics::WIND_SENSORS, QUEUE_SIZE);
            filtered_wind_sensor_pub_ =
              this->create_publisher<msg::WindSensor>(ros_topics::FILTERED_WIND_SENSOR, QUEUE_SIZE);

            can_trns_->registerCanCbs(
              {std::make_pair(
                 CanId::BMS_P_DATA_FRAME_1,
                 std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishBattery(frame); })),
               std::make_pair(
                 CanId::BMS_P_DATA_FRAME_2,
                 std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishBattery(frame); })),
               // TODO(lross03): Add remaining pairs
               std::make_pair(
                 CanId::PATH_GPS_DATA_FRAME_1,
                 std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishGPS(frame); })),
               std::make_pair(
                 CanId::SAIL_WIND_DATA_FRAME_1,
                 std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishWindSensor(frame); })),
               std::make_pair(
                 CanId::PATH_WIND_DATA_FRAME,
                 std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishWindSensor(frame); })),
               std::make_pair(
                 CanId::GENERIC_SENSOR_START,
                 std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishGeneric(frame); }))});

            if (mode == SYSTEM_MODE::DEV) {  // Initialize the CAN Sim Intf
                mock_ais_sub_ = this->create_subscription<msg::AISShips>(
                  ros_topics::MOCK_AIS_SHIPS, QUEUE_SIZE,
                  [this](msg::AISShips mock_ais_ships) { subMockAISCb(mock_ais_ships); });
                mock_gps_sub_ = this->create_subscription<msg::GPS>(
                  ros_topics::MOCK_GPS, QUEUE_SIZE, [this](msg::GPS mock_gps) { subMockGpsCb(mock_gps); });

                // TODO(lross03): register a callback for CanSimToBoatSim

                timer_ = this->create_wall_timer(TIMER_INTERVAL, [this]() {
                    mockBatteriesCb();
                    // Add any other necessary looping callbacks
                });
            }
        }
    }

private:
    // pointer to the CAN Transceiver implementation
    std::unique_ptr<CanTransceiver> can_trns_;

    // Universal publishers and subscribers present in both deployment and simulation
    rclcpp::Publisher<msg::AISShips>::SharedPtr       ais_pub_;
    msg::AISShips                                     ais_ships_;
    rclcpp::Publisher<msg::Batteries>::SharedPtr      batteries_pub_;
    msg::Batteries                                    batteries_;
    rclcpp::Publisher<msg::GPS>::SharedPtr            gps_pub_;
    msg::GPS                                          gps_;
    rclcpp::Publisher<msg::WindSensors>::SharedPtr    wind_sensors_pub_;
    msg::WindSensors                                  wind_sensors_;
    rclcpp::Publisher<msg::WindSensor>::SharedPtr     filtered_wind_sensor_pub_;
    msg::WindSensor                                   filtered_wind_sensor_;
    rclcpp::Publisher<msg::GenericSensors>::SharedPtr generic_sensors_pub_;
    msg::GenericSensors                               generic_sensors_;

    // Simulation only publishers and subscribers
    rclcpp::Subscription<msg::AISShips>::SharedPtr mock_ais_sub_;
    rclcpp::Subscription<msg::GPS>::SharedPtr      mock_gps_sub_;

    // Timer for anything that just needs a repeatedly written value in simulation
    rclcpp::TimerBase::SharedPtr timer_;

    // Mock CAN file descriptor for simulation
    int sim_intf_fd_;

    /**
     * @brief Publish AIS ships
     *
     */
    void publishAIS(const CanFrame & /**/)
    {
        //TODO(): Should be registered with CAN Transceiver once ELEC defines AIS frames
        ais_pub_->publish(ais_ships_);
    }

    /**
     * @brief Publish a battery_frame
     *        Inteneded to be registered as a callback with the CAN Transceiver instance
     *
     * @param battery_frame battery CAN frame read from the CAN bus
     */
    void publishBattery(const CanFrame & battery_frame)
    {
        CAN_FP::Battery bat(battery_frame);

        size_t idx;
        for (size_t i = 0;; i++) {  // idx WILL be in range (can_frame_parser constructors guarantee this)
            if (bat.id_ == CAN_FP::Battery::BATTERY_IDS[i]) {
                idx = i;
                break;
            }
        }
        msg::HelperBattery & bat_msg = batteries_.batteries[idx];
        bat_msg                      = bat.toRosMsg();
        batteries_pub_->publish(batteries_);
    }

    /**
     * @brief Publish a GPS frame
     *        Intended to be registered as a callback with the CAN Tranceiver instance
     *
     * @param gps_frame gps CAN rfame read from the CAN bus
     */
    void publishGPS(const CanFrame & gps_frame)
    {
        CAN_FP::GPS gps(gps_frame);

        msg::GPS gps_ = gps.toRosMsg();
        gps_pub_->publish(gps_);
    }

    void publishWindSensor(const CanFrame & wind_sensor_frame)
    {
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
    }

    void publishFilteredWindSensor()
    {
        int16_t average_direction = 0;
        for (size_t i = 0; i < NUM_WIND_SENSORS; i++) {
            //TODO: check casting later
            average_direction += static_cast<int16_t>(wind_sensors_.wind_sensors[i].direction);  //NOLINT
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
        filtered_wind_sensor_.set__direction(average_direction);

        filtered_wind_sensor_pub_->publish(filtered_wind_sensor_);
    }

    void publishGeneric(const CanFrame & generic_frame)
    {
        //check all generic sensors in the ROS msg for the matching id
        //assumes this sensor is in the generic_sensors_ array of sensors
        size_t idx;
        for (size_t i = 0;; i++) {
            if (generic_frame.can_id == generic_sensors_.generic_sensors[i].id) {
                idx = i;
                break;
            }
        }
        //TODO: ask if i can assume generic_frame is in the generic_sensors array
        uint64_t generic_data = 0;
        std::memcpy(&generic_data, generic_frame.data, sizeof(int64_t));
        msg::HelperGenericSensor & generic_sensor_msg = generic_sensors_.generic_sensors[idx];
        generic_sensor_msg.set__data(generic_data);
        generic_sensor_msg.set__id(generic_frame.can_id);

        generic_sensors_pub_->publish(generic_sensors_);
    }

    /**
     * @brief Mock AIS topic callback
     *
     * @param mock_ais_ships ais_ships received from the Mock AIS topic
     */
    void subMockAISCb(msg::AISShips mock_ais_ships)
    {
        //TODO(): Should be routed through the CAN Transceiver once ELEC defines AIS frames
        ais_ships_ = mock_ais_ships;
        publishAIS(CanFrame{});
    }

    /**
     * @brief Mock GPS topic callback
     *
     * @param mock_gps mock_gps received from the Mock GPS topic
     */
    void subMockGpsCb(msg::GPS /*mock_gps*/)
    {
        // TODO(lross03): implement this
    }

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
        for (size_t i = 0; i < NUM_BATTERIES; i++) {
            auto optCanId = CAN_FP::Battery::rosIdxToCanId(i);
            if (optCanId) {
                can_trns_->send(CAN_FP::Battery(bat, optCanId.value()).toLinuxCan());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send mock battery of index %zu!", i);
            }
        }
    }
};

int main(int argc, char * argv[])
{
    bool err = false;
    rclcpp::init(argc, argv);
    try {
        std::shared_ptr<CanTransceiverIntf> node = std::make_shared<CanTransceiverIntf>();
        try {
            rclcpp::spin(node);
        } catch (std::exception & e) {
            RCLCPP_ERROR(node->get_logger(), "%s", e.what());
            throw e;
        }
    } catch (std::exception & e) {
        std::cerr << e.what() << std::endl;
        err = true;
    }
    rclcpp::shutdown();
    return err ? -1 : 0;
}
