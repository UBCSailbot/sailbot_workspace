#include "can_transceiver.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <stdexcept>
#include <thread>

#include "can_frame_parser.h"
#include "linux/can/raw.h"

using IFreq       = struct ifreq;
using SockAddr    = struct sockaddr;
using SockAddrCan = struct sockaddr_can;

using CAN_FP::CanFrame;
using CAN_FP::CanId;

void CanTransceiver::onNewCanData(const CanFrame & frame) const
{
    if (!CAN_FP::isValidCanId(frame.can_id)) {
        std::cerr << "Invalid CAN ID received: 0x" << std::hex << frame.can_id << std::endl;
        return;
    }
    CanId id{frame.can_id};
    if (read_callbacks_.contains(id)) {
        read_callbacks_.at(id)(frame);  // invoke the callback function mapped to id
    }

    //0x100 - 0x1FF: Generic sensor data frame range
    if (id >= CanId::GENERIC_SENSOR_START && id <= CanId::GENERIC_SENSOR_END) {
        read_callbacks_.at(CanId::GENERIC_SENSOR_START)(frame);
    }
}

void CanTransceiver::registerCanCb(const std::pair<CanId, std::function<void(const CanFrame &)>> cb_kvp)
{
    auto [key, cb]       = cb_kvp;
    read_callbacks_[key] = cb;
}

void CanTransceiver::registerCanCbs(
  const std::vector<std::pair<CanId, std::function<void(const CanFrame &)>>> & cb_kvps)
{
    for (const auto & cb_kvp : cb_kvps) {
        registerCanCb(cb_kvp);
    }
}

CanTransceiver::CanTransceiver() : is_can_simulated_(false)
{
    // See: https://www.kernel.org/doc/html/next/networking/can.html#how-to-use-socketcan
    static const char * CAN_INST = "can0";

    // Everything between this comment and the initiation of the receive thread is pretty much
    // magic from the socketcan documentation

    if ((sock_desc_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::string err_msg = "Failed to open CAN socket with error: " + std::to_string(errno) + ": " +
                              strerror(errno);  // NOLINT(concurrency-mt-unsafe)
        throw std::runtime_error(err_msg);
    }

    IFreq       ifr;
    SockAddrCan addr;

    // Enable reception of CAN FD frames
    {
        int enable = 1;

        if (setsockopt(sock_desc_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
            std::string err_msg = "Failed to set CAN FD";

            throw std::runtime_error(err_msg);
        }
    }

    strncpy(ifr.ifr_name, CAN_INST, IFNAMSIZ);
    ioctl(sock_desc_, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_desc_, reinterpret_cast<const SockAddr *>(&addr), sizeof(addr)) < 0) {
        std::string err_msg = "Failed to bind CAN socket with error: " + std::to_string(errno) + ": " +
                              strerror(errno);  // NOLINT(concurrency-mt-unsafe)

        throw std::runtime_error(err_msg);
    }

    receive_thread_ = std::thread(&CanTransceiver::receive, this);
}

CanTransceiver::CanTransceiver(int fd) : sock_desc_(fd), is_can_simulated_(true)
{
    receive_thread_ = std::thread(&CanTransceiver::receive, this);
}

CanTransceiver::~CanTransceiver()
{
    shutdown_flag_ = true;
    receive_thread_.join();
    close(sock_desc_);
}

void CanTransceiver::receive()
{
    int flags = fcntl(sock_desc_, F_GETFL, 0);
    if (flags == -1) {
        std::cerr << "failed to get flags for CAN socket fd" << std::endl;
    }
    // make read() non-blocking so mutex gets released
    flags |= O_NONBLOCK;
    if (fcntl(sock_desc_, F_SETFL, flags) == -1) {
        std::cerr << "failed to set flags for CAN socket fd" << std::endl;
    }
    while (!shutdown_flag_) {
        // make sure the lock is acquired and released INSIDE the loop, otherwise send() will never get the lock
        CanFrame                     frame;
        std::unique_lock<std::mutex> lock(can_mtx_);
        ssize_t                      bytes_read = read(sock_desc_, &frame, sizeof(CanFrame));
        lock.unlock();
        if (bytes_read > 0) {
            if (bytes_read != sizeof(CanFrame)) {
                std::cerr << "CAN read error: read " << bytes_read << "B but CAN frames are expected to be "
                          << sizeof(CanFrame) << "B" << std::endl;
            } else {
                onNewCanData(frame);
            }
        } else if (bytes_read < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "CAN read error: " << errno << "(" << strerror(errno)  // NOLINT(concurrency-mt-unsafe)
                          << ")" << std::endl;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
}

void CanTransceiver::send(const CanFrame & frame) const
{
    std::lock_guard<std::mutex> lock(can_mtx_);
    ssize_t                     bytes_written = write(sock_desc_, &frame, sizeof(CanFrame));
    if (bytes_written < 0) {
        std::cerr << "CAN write error: " << errno << "(" << strerror(errno)  // NOLINT(concurrency-mt-unsafe)
                  << ")" << std::endl;
    } else {
        if (bytes_written != sizeof(CanFrame)) {
            std::cerr << "CAN write error: wrote " << bytes_written << "B but CAN frames are expected to be "
                      << sizeof(CanFrame) << "B" << std::endl;
        }
        if (is_can_simulated_) {
            // Since we're writing to the same file we're reading from, we need to maintain the seek offset
            // This is NOT necessary in deployment as we won't be using a file to mock it
            lseek(sock_desc_, -static_cast<off_t>(sizeof(CAN_FP::CanFrame)), SEEK_CUR);
        }
    }
}

int mockCanFd(std::string tmp_file_template_str)
{
    // The vector<char> is just super verbose and ugly std::string to cstr conversion done purely because
    // the mkstemp() function is finicky with the string type it wants
    std::vector<char> tmp_file_template_cstr(
      tmp_file_template_str.c_str(), tmp_file_template_str.c_str() + tmp_file_template_str.size() + 1);
    int fd = mkstemp(tmp_file_template_cstr.data());
    if (fd == -1) {
        std::string err_msg = "Failed to open mock CAN fd with error: " + std::to_string(errno) + "(" +
                              strerror(errno) + ")";  // NOLINT(concurrency-mt-unsafe)
        throw std::runtime_error(err_msg);
    }
    return fd;
}
