#pragma once
#include <rclcpp/node.hpp>
/**
 * Network Systems custom ROS Node.
 * Redirects std::cout and std::cerr streams to ROS logging macros.
 * @note An extra newline character is added to ROS logs from redirected streams. Not the prettiest, but it's harmless.
 *       Errors are extra ugly, with multiple empty newlines. It does make them extra obvious though.
 */
class NetNode : public rclcpp::Node
{
public:
    /**
     * @brief Initialize a NetNode's buffers and stream redirections
     *
     * @param node_name Name of the node to pass to the base rclcpp::Node
     */
    explicit NetNode(const std::string & node_name);
    ~NetNode();

private:
    /**
     * @brief Custom string buffer that flushes its contents to ROS logging macros.
     *        https://stackoverflow.com/a/14232652
     */
    class LogBuf : public std::stringbuf
    {
    public:
        /**
         * @brief Initialize a LogBuf for the NetNode
         *
         * @param fd     File descriptor. Must be either STDOUT_FILENO or STDERR_FILENO
         * @param logger Logger object of from the encompassing NetNode
         */
        LogBuf(int fd, rclcpp::Logger logger);

        /**
         * @brief Called when the buffer is flushed. Invokes ROS logging macros and clears buffer.
         *
         * @return 0
         */
        virtual int sync();

    private:
        const int            fd_;      // File descriptor to redirect (either STDOUT_FILENO or STDERR_FILENO)
        const rclcpp::Logger logger_;  // Logger instance from the encompassing NetNode
    };

    std::streambuf * og_stdout_buf_;   // Original buffer for stdout
    std::streambuf * og_stderr_buf_;   // Original buffer for stderr
    LogBuf           new_stdout_buf_;  // LogBuf to redirect stdout to
    LogBuf           new_stderr_buf_;  // LogBuf to redirect stderr to
};
