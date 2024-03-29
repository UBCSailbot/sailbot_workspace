// Include this module
#include "cached_fib.h"
#include "cmn_hdrs/ros_info.h"
#include "net_node.h"
// Include ROS headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace
{
constexpr int ROS_Q_SIZE    = 10;
constexpr int INIT_FIB_SIZE = 5;
}  // namespace

class CachedFibNode : public NetNode
{
public:
    explicit CachedFibNode(const std::size_t initSize) : NetNode(ros_nodes::CACHED_FIB), c_fib_(initSize)
    {
        this->declare_parameter("enabled", false);
        bool enabled = this->get_parameter("enabled").as_bool();
        if (enabled) {
            RCLCPP_INFO(this->get_logger(), "Running example cached fib node");

            // This registers a callback function that gets called whenever CACHED_FIB_TOPIC_IN gets updated
            sub_ = this->create_subscription<std_msgs::msg::UInt64>(
              CACHED_FIB_TOPIC_IN, ROS_Q_SIZE, std::bind(&CachedFibNode::subCb, this, std::placeholders::_1));

            pub_ = this->create_publisher<std_msgs::msg::UInt64>(CACHED_FIB_TOPIC_OUT, ROS_Q_SIZE);
        }
    }

private:
    CachedFib                                              c_fib_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr    pub_;

    /**
     * @brief Callback function that runs whenever the subscribed topic is updated
     *
     * @param in_msg msg that was sent to the topic
     */
    void subCb(const std_msgs::msg::UInt64::SharedPtr in_msg)
    {
        int fibNum = this->c_fib_.getFib(in_msg->data);
        RCLCPP_INFO(this->get_logger(), "Fib num for '%lu' is '%d'", in_msg->data, fibNum);
        std_msgs::msg::UInt64 out_msg{};
        out_msg.data = fibNum;
        pub_->publish(out_msg);
    }
};

int main(int argc, char * argv[])
{
    bool err = false;
    rclcpp::init(argc, argv);
    try {
        std::shared_ptr<CachedFibNode> node = std::make_shared<CachedFibNode>(INIT_FIB_SIZE);
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
