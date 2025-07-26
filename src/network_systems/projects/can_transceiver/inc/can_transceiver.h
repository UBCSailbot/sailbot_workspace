#pragma once

#include <linux/can.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "can_frame_parser.h"

/**
 * @brief CAN Transceiver Class
 *        Handles transmission and reception of data to and from the hardware/simulator
 *
 */
class CanTransceiver
{
public:
    /**
    * @brief Construct a new Can Transceiver and connect it to the default CAN interface (can0)
    * @note  Can only be used in deployment
    *
    */
    CanTransceiver();

    /**
     * @brief Destroy the Canbus Intf object
     *
     */
    ~CanTransceiver();

    /**
     * @brief Construct a new Can Transceiver and connect it to an existing and open file descriptor
     * @note  Can only be used if simulating the CAN bus
     *
     * @param fd
     */
    explicit CanTransceiver(int fd);

    /**
     * @brief Send a CAN frame to the CAN port
     *
     * @param frame CAN frame to send
     */
    void send(const CAN_FP::CanFrame & frame) const;

    /**
     * @brief Register a CanId -> CallbackFunc mapping to be called when the CanId is read from the CAN port
     *
     * @param cb_kvp pair of CanId and the callback function to associate with it
     */
    void registerCanCb(std::pair<CAN_FP::CanId, std::function<void(const CAN_FP::CanFrame &)>> cb_kvp);

    /**
     * @brief Register multiple CanId -> CallbackFunc mappings. See registerCanCb().
     *
     * @param cb_kvps initializer list of cb_kvp pairs
     */
    void registerCanCbs(
      const std::vector<std::pair<CAN_FP::CanId, std::function<void(const CAN_FP::CanFrame &)>>> & cb_kvps);

    /**
     * @brief Set the CAN mode to normal or manual.
     *
     * @param newMode new mode to set to
     */
    static void setCanMode(uint8_t newMode);

private:
    // CAN socket this instance is attached to (can be a normal file descriptor when simulating)
    int sock_desc_;
    // flag to indicate whether the connected CAN socket is a simulated socket or a real socket
    bool is_can_simulated_;
    // Mutex to protect the CAN port from simultaneous reads and writes
    // mutable keyword required for std::lock_guard
    mutable std::mutex can_mtx_;

    // Thread that listens to CAN
    std::thread receive_thread_;
    // Flag to tell the receive_thread_ to stop
    bool shutdown_flag_ = false;

    // For each CanId key in this map, if the CanId is read from the CAN bus, then the associated callback function
    // is invoked
    std::map<CAN_FP::CanId, std::function<void(const CAN_FP::CanFrame &)>> read_callbacks_;

    /**
     * @brief Retrieve latest incoming CAN frame from hardware and process it
     *        Infinitely loops on a read() syscall, so needs to be run in another thread
     *        Can be shutdown by setting shutdown_flag_ to true
     */
    void receive();
    /**
     * @brief Call on successfully reading a new CAN data frame from hardware/simulator
     *
     * @param frame received CAN data frame
     */
    void onNewCanData(const CAN_FP::CanFrame & frame) const;
};

/**
 * @brief Create a mock CAN socket descriptor for simulation purposes
 *
 * @param template_str File path ending in XXXXXX. Ex: "/tmp/fileXXXXXX"
 * @return int opened file descriptor
 */
int mockCanFd(std::string template_str);
