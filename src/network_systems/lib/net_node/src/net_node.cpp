#include "net_node.h"

#include <rclcpp/rclcpp.hpp>

NetNode::NetNode(const std::string & node_name)
: rclcpp::Node(node_name),
  // When constructing the new buffers, create a new rclcpp::Logger with ":net_node" appended to the name so that
  // we can identify that redirected messages are output from this file, rather than the _ros_intf.cpp file.
  // Hopefully this will help reduce confusion with respect to the printed line numbers.
  new_stdout_buf_(STDOUT_FILENO, rclcpp::get_logger(std::string(this->get_logger().get_name()) + ":net_node")),
  new_stderr_buf_(STDERR_FILENO, rclcpp::get_logger(std::string(this->get_logger().get_name()) + ":net_node"))
{
    og_stdout_buf_ = std::cout.rdbuf();
    og_stderr_buf_ = std::cerr.rdbuf();
    std::cout.rdbuf(&new_stdout_buf_);
    std::cerr.rdbuf(&new_stderr_buf_);
}

NetNode::~NetNode()
{
    std::cout.rdbuf(og_stdout_buf_);
    std::cerr.rdbuf(og_stderr_buf_);
}

NetNode::LogBuf::LogBuf(int fd, rclcpp::Logger logger) : fd_(fd), logger_(logger) {}

int NetNode::LogBuf::sync()
{
    switch (fd_) {
        case STDOUT_FILENO:
            RCLCPP_INFO(logger_, "%s", this->str().c_str());
            break;
        case STDERR_FILENO:
            RCLCPP_ERROR(logger_, "%s", this->str().c_str());
            break;
        default:
            // unreachable
            throw std::runtime_error("Invalid file descriptor! " + std::to_string(fd_));
    }
    this->str("");
    return 0;
}
