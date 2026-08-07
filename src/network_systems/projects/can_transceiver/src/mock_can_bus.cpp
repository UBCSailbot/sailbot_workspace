#include "mock_can_bus.h"

#include <errno.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>

using CAN_FP::CanFrame;
using CAN_REPLAY::CanLogReplayer;

namespace
{
constexpr auto STOP_CHECK_SLICE  = std::chrono::milliseconds(5);   // max sleep between stop flag checks
constexpr int  SINK_POLL_TIMEOUT = 10;                             // ms to block waiting for outbound frames
constexpr auto WRITE_RETRY_DELAY = std::chrono::milliseconds(1);   // wait before retrying a full socket buffer
}  // namespace

MockCanBus::MockCanBus()
{
    std::array<int, 2> fds{};
    // SOCK_SEQPACKET reads each write as one whole datagram, mirroring real CAN socket semantics
    if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, fds.data()) < 0) {
        std::string err_msg = "Failed to create mock CAN bus socketpair with error: " + std::to_string(errno) + "(" +
                              strerror(errno) + ")";  // NOLINT(concurrency-mt-unsafe)
        throw std::runtime_error(err_msg);
    }
    transceiver_fd_ = fds[0];
    bus_fd_         = fds[1];

    sink_thread_ = std::thread(&MockCanBus::sinkLoop, this);
}

MockCanBus::~MockCanBus()
{
    stopReplay();
    stop_sink_ = true;
    if (sink_thread_.joinable()) {
        sink_thread_.join();
    }
    close(bus_fd_);
    // transceiver_fd_ is intentionally left open - the CanTransceiver it was handed to owns it
}

void MockCanBus::startReplay(std::vector<CAN_REPLAY::TimedFrame> frames, CAN_REPLAY::ReplayConfig cfg)
{
    stopReplay();

    frames_      = CanLogReplayer::filter(std::move(frames), cfg);
    cfg_         = cfg;
    stop_replay_ = false;
    replay_done_ = false;
    frames_sent_ = 0;

    replay_thread_ = std::thread(&MockCanBus::replayLoop, this);
}

void MockCanBus::stopReplay()
{
    stop_replay_ = true;
    if (replay_thread_.joinable()) {
        replay_thread_.join();
    }
}

std::vector<CanFrame> MockCanBus::outboundFrames() const
{
    std::lock_guard<std::mutex> lock(outbound_mtx_);
    return outbound_;
}

void MockCanBus::replayLoop()
{
    do {
        for (size_t i = 0; i < frames_.size() && !stop_replay_; i++) {
            auto remaining = CanLogReplayer::delayBefore(frames_, i, cfg_);
            while (remaining > std::chrono::nanoseconds{0} && !stop_replay_) {
                auto slice = std::min<std::chrono::nanoseconds>(remaining, STOP_CHECK_SLICE);
                std::this_thread::sleep_for(slice);
                remaining -= slice;
            }
            if (stop_replay_) {
                break;
            }
            if (!writeFrame(frames_[i].frame)) {
                return;
            }
            frames_sent_++;
        }
    } while (cfg_.loop && !stop_replay_);
    // a stopped replay is not done, and stopping is the only way a looping replay exits
    replay_done_ = !stop_replay_;
}

bool MockCanBus::writeFrame(const CanFrame & frame)
{
    while (!stop_replay_) {
        // MSG_NOSIGNAL: if the CanTransceiver end is already closed, fail with EPIPE instead of raising SIGPIPE
        ssize_t bytes_written = ::send(bus_fd_, &frame, sizeof(CanFrame), MSG_DONTWAIT | MSG_NOSIGNAL);
        if (bytes_written == sizeof(CanFrame)) {
            return true;
        }
        if (bytes_written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            std::this_thread::sleep_for(WRITE_RETRY_DELAY);
            continue;
        }
        // single << so log frameworks capturing stderr do not split the message
        std::cerr << ("Mock CAN bus write error: " + std::to_string(errno) + "(" +
                      strerror(errno) + ")\n");  // NOLINT(concurrency-mt-unsafe)
        return false;
    }
    return false;
}

void MockCanBus::sinkLoop()
{
    struct pollfd pfd;
    pfd.fd     = bus_fd_;
    pfd.events = POLLIN;

    while (!stop_sink_) {
        int ready = poll(&pfd, 1, SINK_POLL_TIMEOUT);
        if (ready <= 0) {
            continue;
        }
        CanFrame frame;
        ssize_t  bytes_read = recv(bus_fd_, &frame, sizeof(CanFrame), MSG_DONTWAIT);
        if (bytes_read == sizeof(CanFrame)) {
            std::lock_guard<std::mutex> lock(outbound_mtx_);
            outbound_.push_back(frame);
        } else if (bytes_read == 0) {
            return;  // CanTransceiver end closed - nothing left to capture
        } else if (bytes_read > 0) {
            std::cerr << ("Mock CAN bus read error: read " + std::to_string(bytes_read) +
                          "B but CAN frames are expected to be " + std::to_string(sizeof(CanFrame)) + "B\n");
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            std::cerr << ("Mock CAN bus read error: " + std::to_string(errno) + "(" +
                          strerror(errno) + ")\n");  // NOLINT(concurrency-mt-unsafe)
        }
    }
}
