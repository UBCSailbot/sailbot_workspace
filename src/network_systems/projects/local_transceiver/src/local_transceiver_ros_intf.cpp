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
 * Local Transceiver Interface Node
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
    explicit LocalTransceiverIntf(std::shared_ptr<LocalTransceiver> lcl_trns)
    : NetNode("local_transceiver_node"), lcl_trns_(lcl_trns)
    {
        static constexpr int  ROS_Q_SIZE     = 5;
        static constexpr auto TIMER_INTERVAL = std::chrono::milliseconds(500);
        timer_ = this->create_wall_timer(TIMER_INTERVAL, std::bind(&LocalTransceiverIntf::pub_cb, this));
        pub_   = this->create_publisher<custom_interfaces::msg::Path>(GLOBAL_PATH_TOPIC, ROS_Q_SIZE);

        // subscriber nodes
        sub_wind_sensor = this->create_subscription<custom_interfaces::msg::WindSensors>(
          WIND_SENSORS_TOPIC, ROS_Q_SIZE,
          std::bind(&LocalTransceiverIntf::sub_wind_sensor_cb, this, std::placeholders::_1));
        sub_batteries = this->create_subscription<custom_interfaces::msg::Batteries>(
          BATTERIES_TOPIC, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_batteries_cb, this, std::placeholders::_1));
        sub_data_sensors = this->create_subscription<custom_interfaces::msg::GenericSensors>(
          DATA_SENSORS_TOPIC, ROS_Q_SIZE,
          std::bind(&LocalTransceiverIntf::sub_data_sensors_cb, this, std::placeholders::_1));
        sub_ais_ships = this->create_subscription<custom_interfaces::msg::AISShips>(
          AIS_SHIPS_TOPIC, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_ais_ships_cb, this, std::placeholders::_1));
        sub_gps = this->create_subscription<custom_interfaces::msg::GPS>(
          GPS_TOPIC, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_gps_cb, this, std::placeholders::_1));
        sub_local_path_data = this->create_subscription<custom_interfaces::msg::LPathData>(
          LOCAL_PATH_DATA_TOPIC, ROS_Q_SIZE,
          std::bind(&LocalTransceiverIntf::sub_local_path_data_cb, this, std::placeholders::_1));
    }

private:
    // Local Transceiver instance
    std::shared_ptr<LocalTransceiver> lcl_trns_;
    // Publishing timer
    rclcpp::TimerBase::SharedPtr timer_;
    // String is a placeholder pub and sub msg type - we will definitely define custom message types
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
    void pub_cb(/*place*/)
    {
        // TODO(Jng468): complete, after receive is done
        // std::string recent_data = lcl_trns_->receive();  //receives most recent data from remote server
        // auto msg = custom_interfaces::msg::Path();
        // pub_->publish(msg);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for wind sensors
     */
    void sub_wind_sensor_cb(custom_interfaces::msg::WindSensors in_msg)
    {
        custom_interfaces::msg::WindSensors data = in_msg;
        lcl_trns_->updateSensor(data);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for batteries
     */
    void sub_batteries_cb(custom_interfaces::msg::Batteries in_msg)
    {
        custom_interfaces::msg::Batteries data = in_msg;
        lcl_trns_->updateSensor(data);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for generic sensors
     */
    void sub_data_sensors_cb(custom_interfaces::msg::GenericSensors in_msg)
    {
        custom_interfaces::msg::GenericSensors data = in_msg;
        lcl_trns_->updateSensor(data);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for ais ships
     */
    void sub_ais_ships_cb(custom_interfaces::msg::AISShips in_msg)
    {
        custom_interfaces::msg::AISShips data = in_msg;
        lcl_trns_->updateSensor(data);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for GPS
     */
    void sub_gps_cb(custom_interfaces::msg::GPS in_msg)
    {
        custom_interfaces::msg::GPS data = in_msg;
        lcl_trns_->updateSensor(data);
    }

    void sub_local_path_data_cb(custom_interfaces::msg::LPathData in_msg)
    {
        custom_interfaces::msg::LPathData data = in_msg;
        lcl_trns_->updateSensor(data);
    }
};

int main(int argc, char * argv[])
{
    bool err = false;
    rclcpp::init(argc, argv);
    std::shared_ptr<LocalTransceiver> lcl_trns = std::make_shared<LocalTransceiver>("PLACEHOLDER", SATELLITE_BAUD_RATE);
    try {
        std::shared_ptr<LocalTransceiverIntf> node = std::make_shared<LocalTransceiverIntf>(lcl_trns);
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
