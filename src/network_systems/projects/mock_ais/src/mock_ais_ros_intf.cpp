#include <chrono>
#include <custom_interfaces/msg/ais_ships.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/wait_for_message.hpp>

#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "mock_ais.h"
#include "net_node.h"

/**
 * Connect the Mock AIS to the onbaord ROS network
 */
class MockAisRosIntf : public NetNode
{
public:
    MockAisRosIntf() : NetNode(ros_nodes::MOCK_AIS)
    {
        static constexpr int ROS_Q_SIZE = 5;
        this->declare_parameter("enabled", false);

        if (this->get_parameter("enabled").as_bool()) {
            this->declare_parameter("mode", rclcpp::PARAMETER_STRING);
            this->declare_parameter("publish_rate_ms", defaults::UPDATE_RATE_MS);
            this->declare_parameter("seed", defaults::SEED);
            this->declare_parameter("num_sim_ships", defaults::NUM_SIM_SHIPS);
            this->declare_parameter(
              "polaris_start_pos",
              std::vector<float>({defaults::POLARIS_START_POS[0], defaults::POLARIS_START_POS[1]}));

            rclcpp::Parameter mode_param              = this->get_parameter("mode");
            rclcpp::Parameter publish_rate_ms_param   = this->get_parameter("publish_rate_ms");
            rclcpp::Parameter seed_param              = this->get_parameter("seed");
            rclcpp::Parameter num_sim_ships_param     = this->get_parameter("num_sim_ships");
            rclcpp::Parameter polaris_start_pos_param = this->get_parameter("polaris_start_pos");

            std::string mode              = mode_param.as_string();
            int64_t     publish_rate_ms   = publish_rate_ms_param.as_int();
            int64_t     seed              = seed_param.as_int();
            int64_t     num_sim_ships     = num_sim_ships_param.as_int();
            Vec2DFloat  polaris_start_pos = {// annoyingly ugly type conversion :/
                                            static_cast<float>(polaris_start_pos_param.as_double_array()[0]),
                                            static_cast<float>(polaris_start_pos_param.as_double_array()[1])};

            RCLCPP_INFO(
              this->get_logger(),
              "Running Mock AIS in mode: %s, with publish_rate_ms: %s, seed: %s, num_ships %s, polaris_start_pos: %s",
              mode.c_str(), publish_rate_ms_param.value_to_string().c_str(), seed_param.value_to_string().c_str(),
              num_sim_ships_param.value_to_string().c_str(), polaris_start_pos_param.value_to_string().c_str());

            // TODO(): Add ROS parameters so that we can use the MockAis constructor that takes SimShipConfig
            // Optionally use nested parameters: https://answers.ros.org/question/325939/declare-nested-parameter/
            mock_ais_                     = std::make_unique<MockAis>(seed, num_sim_ships, polaris_start_pos);
            std::string polaris_gps_topic = mode == SYSTEM_MODE::DEV ? ros_topics::MOCK_GPS : ros_topics::GPS;

            // The subscriber callback is very simple so it's just the following lambda function
            sub_ = this->create_subscription<custom_interfaces::msg::GPS>(
              polaris_gps_topic, ROS_Q_SIZE, [&mock_ais_ = mock_ais_](custom_interfaces::msg::GPS mock_gps) {
                  mock_ais_->updatePolarisPos({mock_gps.lat_lon.latitude, mock_gps.lat_lon.longitude});
              });

            pub_   = this->create_publisher<custom_interfaces::msg::AISShips>(ros_topics::MOCK_AIS_SHIPS, ROS_Q_SIZE);
            timer_ = this->create_wall_timer(
              std::chrono::milliseconds(publish_rate_ms), std::bind(&MockAisRosIntf::pubShipsCB, this));
        } else {
            RCLCPP_INFO(this->get_logger(), "Mock AIS is DISABLED");
        }
    }

private:
    std::unique_ptr<MockAis>                                       mock_ais_;  // Mock AIS instance
    rclcpp::TimerBase::SharedPtr                                   timer_;     // publish timer
    rclcpp::Publisher<custom_interfaces::msg::AISShips>::SharedPtr pub_;       // Publish new AISShips info
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr   sub_;       // Subscribe to Polaris' GPS coordinates

    /**
     * @brief Publish the latest mock ais ships data
     *
     */
    void pubShipsCB()
    {
        mock_ais_->tick();
        std::vector<AisShip>             ais_ships = mock_ais_->ships();
        custom_interfaces::msg::AISShips msg{};
        for (const AisShip & ais_ship : ais_ships) {
            custom_interfaces::msg::HelperAISShip helper_ship;
            helper_ship.set__id(static_cast<int32_t>(ais_ship.id_));
            custom_interfaces::msg::HelperHeading helper_head;
            helper_head.set__heading(ais_ship.heading_);
            helper_ship.set__cog(helper_head);
            custom_interfaces::msg::HelperSpeed helper_speed;
            helper_speed.set__speed(ais_ship.speed_);
            helper_ship.set__sog(helper_speed);
            custom_interfaces::msg::HelperLatLon lat_lon;
            lat_lon.set__latitude(ais_ship.lat_lon_[0]);
            lat_lon.set__longitude(ais_ship.lat_lon_[1]);
            helper_ship.set__lat_lon(lat_lon);
            custom_interfaces::msg::HelperDimension width;
            width.set__dimension(static_cast<float>(ais_ship.width_));
            helper_ship.set__width(width);
            custom_interfaces::msg::HelperDimension length;
            length.set__dimension(static_cast<float>(ais_ship.length_));
            helper_ship.set__length(length);
            custom_interfaces::msg::HelperROT rot;
            rot.set__rot(ais_ship.rot_);
            helper_ship.set__rot(rot);

            msg.ships.push_back(helper_ship);
        }
        pub_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    bool err = false;
    rclcpp::init(argc, argv);
    try {
        std::shared_ptr<MockAisRosIntf> node = std::make_shared<MockAisRosIntf>();
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
