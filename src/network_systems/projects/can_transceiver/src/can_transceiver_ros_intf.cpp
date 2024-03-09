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
    CanTransceiverIntf() : NetNode("can_transceiver_node")
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

            ais_pub_       = this->create_publisher<msg::AISShips>(AIS_SHIPS_TOPIC, QUEUE_SIZE);
            batteries_pub_ = this->create_publisher<msg::Batteries>(BATTERIES_TOPIC, QUEUE_SIZE);

            can_trns_->registerCanCbs({
              std::make_pair(
                CanId::BMS_P_DATA_FRAME_1,
                std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishBattery(frame); })),
              std::make_pair(
                CanId::BMS_P_DATA_FRAME_2,
                std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishBattery(frame); })),
              // TODO(lross03): Add remaining pairs
            });

            if (mode == SYSTEM_MODE::DEV) {  // Initialize the CAN Sim Intf
                mock_ais_sub_ = this->create_subscription<msg::AISShips>(
                  MOCK_AIS_SHIPS_TOPIC, QUEUE_SIZE,
                  [this](msg::AISShips mock_ais_ships) { subMockAISCb(mock_ais_ships); });
                mock_gps_sub_ = this->create_subscription<msg::GPS>(
                  MOCK_GPS_TOPIC, QUEUE_SIZE, [this](msg::GPS mock_gps) { subMockGpsCb(mock_gps); });

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
    rclcpp::Publisher<msg::AISShips>::SharedPtr  ais_pub_;
    msg::AISShips                                ais_ships_;
    rclcpp::Publisher<msg::Batteries>::SharedPtr batteries_pub_;
    msg::Batteries                               batteries_;

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
