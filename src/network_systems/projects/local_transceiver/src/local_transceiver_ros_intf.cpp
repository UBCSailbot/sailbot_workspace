#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>

#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "local_transceiver.h"
#include "net_node.h"

/**
 * @brief Connect the Local Transceiver to the ROS network
 *
 */
class LocalTransceiverIntf : public NetNode
{
public:
    /**
     * @brief Construct a new Local Transceiver Intf Node
     *
     * @param lcl_trns Local Transceiver instance
     */
    explicit LocalTransceiverIntf() : NetNode("local_transceiver_node")

    {
        this->declare_parameter("enabled", true);
        bool enabled_ = this->get_parameter("enabled").as_bool();

        if (!enabled_) {
            RCLCPP_INFO(this->get_logger(), "Local Transceiver is DISABLED");
        } else {
            this->declare_parameter("mode", SYSTEM_MODE::DEV);

            rclcpp::Parameter mode_param = this->get_parameter("mode");
            std::string       mode       = mode_param.as_string();
            std::string       default_port;

            if (mode == SYSTEM_MODE::PROD) {
                //TODO(Jng468) placeholder
            } else if (mode == SYSTEM_MODE::DEV) {
                default_port = LOCAL_TRANSCEIVER_TEST_PORT;
            } else {
                std::string msg = "Error, invalid system mode" + mode;
                throw std::runtime_error(msg);
            }

            this->declare_parameter("port", default_port);
            rclcpp::Parameter default_port_parm = this->get_parameter("port");
            std::string       port              = default_port_parm.as_string();

            RCLCPP_INFO(
              this->get_logger(), "Running Local Transceiver in mode: %s, with port: %s.", mode.c_str(), port.c_str());
            lcl_trns_ = std::make_unique<LocalTransceiver>(port, SATELLITE_BAUD_RATE);

            static constexpr int  ROS_Q_SIZE     = 5;
            static constexpr auto TIMER_INTERVAL = std::chrono::milliseconds(300000);
            timer_ = this->create_wall_timer(TIMER_INTERVAL, std::bind(&LocalTransceiverIntf::pub_cb, this));
            pub_   = this->create_publisher<custom_interfaces::msg::Path>(ros_topics::GLOBAL_PATH, ROS_Q_SIZE);

            // subscriber nodes
            sub_wind_sensor = this->create_subscription<custom_interfaces::msg::WindSensors>(
              ros_topics::WIND_SENSORS, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_wind_sensor_cb, this, std::placeholders::_1));
            sub_batteries = this->create_subscription<custom_interfaces::msg::Batteries>(
              ros_topics::BATTERIES, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_batteries_cb, this, std::placeholders::_1));
            sub_data_sensors = this->create_subscription<custom_interfaces::msg::GenericSensors>(
              ros_topics::DATA_SENSORS, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_data_sensors_cb, this, std::placeholders::_1));
            sub_ais_ships = this->create_subscription<custom_interfaces::msg::AISShips>(
              ros_topics::AIS_SHIPS, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_ais_ships_cb, this, std::placeholders::_1));
            sub_gps = this->create_subscription<custom_interfaces::msg::GPS>(
              ros_topics::GPS, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_gps_cb, this, std::placeholders::_1));
            sub_local_path_data = this->create_subscription<custom_interfaces::msg::LPathData>(
              ros_topics::LOCAL_PATH, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_local_path_data_cb, this, std::placeholders::_1));
        }
    }

private:
    std::unique_ptr<LocalTransceiver>                          lcl_trns_;  // Local Transceiver instance
    rclcpp::TimerBase::SharedPtr                               timer_;     // Publishing timer
    rclcpp::Publisher<custom_interfaces::msg::Path>::SharedPtr pub_;

    rclcpp::Subscription<custom_interfaces::msg::WindSensors>::SharedPtr    sub_wind_sensor;
    rclcpp::Subscription<custom_interfaces::msg::Batteries>::SharedPtr      sub_batteries;
    rclcpp::Subscription<custom_interfaces::msg::GenericSensors>::SharedPtr sub_data_sensors;
    rclcpp::Subscription<custom_interfaces::msg::AISShips>::SharedPtr       sub_ais_ships;
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr            sub_gps;
    rclcpp::Subscription<custom_interfaces::msg::LPathData>::SharedPtr      sub_local_path_data;

    /**
     * @brief Callback function to publish to onboard ROS network
     *
     */
    void pub_cb(/*placeholder*/)
    {
        custom_interfaces::msg::Path msg = lcl_trns_->receive();
        pub_->publish(msg);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for wind sensors
     */
    void sub_wind_sensor_cb(custom_interfaces::msg::WindSensors in_msg) { lcl_trns_->updateSensor(in_msg); }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for batteries
     */
    void sub_batteries_cb(custom_interfaces::msg::Batteries in_msg) { lcl_trns_->updateSensor(in_msg); }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for generic sensors
     */
    void sub_data_sensors_cb(custom_interfaces::msg::GenericSensors in_msg) { lcl_trns_->updateSensor(in_msg); }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for ais ships
     */
    void sub_ais_ships_cb(custom_interfaces::msg::AISShips in_msg) { lcl_trns_->updateSensor(in_msg); }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for GPS
     */
    void sub_gps_cb(custom_interfaces::msg::GPS in_msg) { lcl_trns_->updateSensor(in_msg); }

    void sub_local_path_data_cb(custom_interfaces::msg::LPathData in_msg) { lcl_trns_->updateSensor(in_msg); }
};

int main(int argc, char * argv[])
{
    bool err = false;
    rclcpp::init(argc, argv);

    try {
        std::shared_ptr<LocalTransceiverIntf> node = std::make_shared<LocalTransceiverIntf>();
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
