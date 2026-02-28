#include <bits/stdc++.h>

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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
                if (mode == SYSTEM_MODE::PROD) {
                    default_port             = "/dev/ttyS0";
                    std::string set_baud_cmd = "stty -F /dev/ttyS0 19200";
                    int         result       = std::system(set_baud_cmd.c_str());  //NOLINT(concurrency-mt-unsafe)
                    if (result != 0) {
                        std::string msg = "Error: could not set baud rate for local trns port /dev/ttyS0";
                        std::cerr << msg << std::endl;
                        throw std::exception();
                    }
                }
            } else if (mode == SYSTEM_MODE::DEV) {
                default_port                = LOCAL_TRANSCEIVER_TEST_PORT;
                std::string run_iridium_cmd = "$ROS_WORKSPACE/scripts/run_virtual_iridium.sh";
                std::thread vi_thread(std::system, run_iridium_cmd.c_str());
                //vi needs to run in background
                vi_thread.detach();

                const int   MAX_ATTEMPTS   = 100;  // 100ms timeout, should only take <5
                int         attempts       = 0;
                const int   SLEEP_MS       = 1;
                const int   IOCTL_ERR_CODE = 256;
                std::string set_baud_cmd   = "stty 19200 < $LOCAL_TRANSCEIVER_TEST_PORT 2>/dev/null";
                //silence stderr to not clutter console while polling
                while (attempts < MAX_ATTEMPTS) {
                    if (std::filesystem::exists(LOCAL_TRANSCEIVER_TEST_PORT)) {
                        int result = std::system(set_baud_cmd.c_str());  //NOLINT(concurrency-mt-unsafe)
                        if (result == 0) {
                            break;
                        }
                        if (result != IOCTL_ERR_CODE) {
                            std::cerr << "Failed to set baud rate with code: " << result << std::endl;
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
                    attempts++;
                }

                if (attempts == MAX_ATTEMPTS) {
                    std::string msg = "Error: could not start virtual iridium";
                    std::cerr << msg << std::endl;
                    throw std::exception();
                }
            } else if (mode == SYSTEM_MODE::TEST_SAT) {
                default_port             = "/dev/ttyS0";
                std::string set_baud_cmd = "stty -F /dev/ttyS0 19200";
                int         result       = std::system(set_baud_cmd.c_str());  //NOLINT(concurrency-mt-unsafe)
                if (result != 0) {
                    std::string msg = "Error: could not set baud rate for local trns port /dev/ttyS0";
                    std::cerr << msg << std::endl;
                    throw std::exception();
                }

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

            // Set up logging callbacks to use ROS logging
            // These are callbacks so that the LocalTransceiver class does not need to make direct ROS calls
            lcl_trns_->setLogCallbacks(
              [this](const std::string & msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); },
              [this](const std::string & msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); });

            std::future<std::optional<custom_interfaces::msg::Path>> fut =
              std::async(std::launch::async, lcl_trns_->getCache);

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

            sub_temp_sensor = this->create_subscription<custom_interfaces::msg::TempSensors>(
              ros_topics::TEMP_SENSORS, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_temp_sensor_cb, this, std::placeholders::_1));
            sub_ph_sensor = this->create_subscription<custom_interfaces::msg::PhSensors>(
              ros_topics::PH_SENSORS, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_ph_sensor_cb, this, std::placeholders::_1));
            sub_salinity_sensor = this->create_subscription<custom_interfaces::msg::SalinitySensors>(
              ros_topics::SALINITY_SENSORS, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_salinity_sensor_cb, this, std::placeholders::_1));

            // sub_pressure_sensor = this->create_subscription<custom_interfaces::msg::PressureSensors>(
            //   ros_topics::PRESSURE_SENSORS, ROS_Q_SIZE,
            //   std::bind(&LocalTransceiverIntf::sub_pressure_sensor_cb, this, std::placeholders::_1));

            sub_gps = this->create_subscription<custom_interfaces::msg::GPS>(
              ros_topics::GPS, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_gps_cb, this, std::placeholders::_1));
            sub_local_path_data = this->create_subscription<custom_interfaces::msg::LPathData>(
              ros_topics::LOCAL_PATH, ROS_Q_SIZE,
              std::bind(&LocalTransceiverIntf::sub_local_path_data_cb, this, std::placeholders::_1));

            std::optional<custom_interfaces::msg::Path> msg = fut.get();
            if (msg) {
                pub_->publish(*msg);
            }

            srv_send_ = this->create_service<std_srvs::srv::Trigger>(
              "send_data", std::bind(
                             &LocalTransceiverIntf::send_request_handler, this, std::placeholders::_1,
                             std::placeholders::_2, false));
            RCLCPP_INFO(this->get_logger(), "send_data service created");

            this->declare_parameter<std::string>("debug_data_to_send", "Hello!");

            srv_send_debug_ = this->create_service<std_srvs::srv::Trigger>(
              "debug_send_data",
              std::bind(
                &LocalTransceiverIntf::send_request_handler, this, std::placeholders::_1, std::placeholders::_2, true));
            RCLCPP_INFO(this->get_logger(), "debug_send_data service created");

            srv_check_signal_quality_ = this->create_service<std_srvs::srv::Trigger>(
              "check_signal_quality", std::bind(
                                        &LocalTransceiverIntf::check_signal_quality_request_handler, this,
                                        std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "check_signal_quality service created");

            srv_receive_and_pub_ = this->create_service<std_srvs::srv::Trigger>(
              "receive_and_publish", std::bind(
                                       &LocalTransceiverIntf::receive_and_pub_request_handler, this,
                                       std::placeholders::_1, std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "receive_and_publish service created");
        }
    }

private:
    std::unique_ptr<LocalTransceiver>                          lcl_trns_;  // Local Transceiver instance
    rclcpp::TimerBase::SharedPtr                               timer_;     // Publishing timer
    rclcpp::Publisher<custom_interfaces::msg::Path>::SharedPtr pub_;

    rclcpp::Subscription<custom_interfaces::msg::WindSensors>::SharedPtr     sub_wind_sensor;
    rclcpp::Subscription<custom_interfaces::msg::Batteries>::SharedPtr       sub_batteries;
    rclcpp::Subscription<custom_interfaces::msg::TempSensors>::SharedPtr     sub_temp_sensor;
    rclcpp::Subscription<custom_interfaces::msg::PhSensors>::SharedPtr       sub_ph_sensor;
    rclcpp::Subscription<custom_interfaces::msg::SalinitySensors>::SharedPtr sub_salinity_sensor;
    // rclcpp::Subscription<custom_interfaces::msg::PressureSensors>::SharedPtr sub_pressure_sensor;
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr       sub_gps;
    rclcpp::Subscription<custom_interfaces::msg::LPathData>::SharedPtr sub_local_path_data;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_send_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_send_debug_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_check_signal_quality_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_receive_and_pub_;

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
     * @brief Callback function to subscribe to the onboard ROS network for temperature sensors
     */
    void sub_temp_sensor_cb(custom_interfaces::msg::TempSensors in_msg) { lcl_trns_->updateSensor(in_msg); }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for pH sensors
     */
    void sub_ph_sensor_cb(custom_interfaces::msg::PhSensors in_msg) { lcl_trns_->updateSensor(in_msg); }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for salinity sensors
     */
    void sub_salinity_sensor_cb(custom_interfaces::msg::SalinitySensors in_msg) { lcl_trns_->updateSensor(in_msg); }

    // /**
    //  * @brief Callback function to subscribe to the onboard ROS network for pressure sensors
    //  */
    // void sub_pressure_sensor_cb(custom_interfaces::msg::PressureSensors in_msg) { lcl_trns_->updateSensor(in_msg); }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for GPS
     */
    void sub_gps_cb(custom_interfaces::msg::GPS in_msg) { lcl_trns_->updateSensor(in_msg); }

    void sub_local_path_data_cb(custom_interfaces::msg::LPathData in_msg) { lcl_trns_->updateSensor(in_msg); }

    void receive_and_pub_request_handler(
      std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        try {
            pub_cb();
            RCLCPP_INFO(
              this->get_logger(), "Receive and pub completed, check if path message was published successfully.");
            response->success = true;
            response->message = "Receive and publish completed";

        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during pub_cb(receive and publish): %s", e.what());
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
        }
    }

    void check_signal_quality_request_handler(
      std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        try {
            int signal_quality = lcl_trns_->checkIridiumSignalQuality();

            if (signal_quality != -1) {
                RCLCPP_INFO(this->get_logger(), "Signal quality: %d", signal_quality);
                response->success = true;
                response->message = "Current Signal Quality: " + std::to_string(signal_quality);
            } else {
                RCLCPP_INFO(
                  this->get_logger(), "Check signal quality unsuccessful: some error occurred in the function body");
                response->success = false;
                response->message = "Check Signal Quality Failed";
            }

        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during checkIridiumSignalQuality(): %s", e.what());
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
        }
    }

    void send_request_handler(
      std::shared_ptr<std_srvs::srv::Trigger::Request>  request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response, bool is_debug = false)
    {
        (void)request;

        try {
            bool success;

            if (is_debug) {
                // Retrieve the debug_data_to_send parameter dynamically
                std::string debug_data = this->get_parameter("debug_data_to_send").as_string();
                success                = lcl_trns_->debugSendAT(debug_data);
            } else {
                success = lcl_trns_->send();
            }

            if (success) {
                RCLCPP_INFO(this->get_logger(), "Successfully sent data via Local Transceiver");
                response->success = true;
                response->message = "Transmission Successful";
            } else {
                RCLCPP_INFO(this->get_logger(), "Send is unsuccessful");
                response->success = false;
                response->message = "Transmission Failed";
            }

        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during send(): %s", e.what());
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
        }
    }
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
