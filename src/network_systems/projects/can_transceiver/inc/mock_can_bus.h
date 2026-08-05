#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "can_frame_parser.h"
#include "can_log_replayer.h"

/**
 * @brief Simulated CAN bus backed by a Unix SOCK_SEQPACKET socketpair
 *        One end is handed to a CanTransceiver (see transceiverFd()) and behaves like a real CAN socket.
 *        The bus end replays logged frames at a configurable pace and captures every frame the
 *        CanTransceiver sends, so tests can inspect the boat's outbound traffic.
 *
 */
class MockCanBus
{
public:
    /**
     * @brief Create the socketpair and start capturing outbound frames
     *
     * @throws std::runtime_error if the socketpair cannot be created
     */
    MockCanBus();

    /**
     * @brief Stop the replay and capture threads and close the bus end of the socketpair
     *
     */
    ~MockCanBus();

    MockCanBus(const MockCanBus &)             = delete;
    MockCanBus & operator=(const MockCanBus &) = delete;

    /**
     * @brief File descriptor for the CanTransceiver end of the socketpair
     * @note  CanTransceiver(int fd) takes ownership and closes it on destruction
     *
     * @return open socket file descriptor
     */
    int transceiverFd() const { return transceiver_fd_; }

    /**
     * @brief Start replaying frames onto the bus in a background thread
     * @note  The config's allow/block lists are applied before replay starts
     *
     * @param frames frames to replay, in order
     * @param cfg    pacing, looping, and filtering configuration
     */
    void startReplay(std::vector<CAN_REPLAY::TimedFrame> frames, CAN_REPLAY::ReplayConfig cfg = {});

    /**
     * @brief Stop an in-progress replay and wait for the replay thread to exit
     *
     */
    void stopReplay();

    /**
     * @return whether the replay thread has sent every frame (never true while looping)
     */
    bool replayDone() const { return replay_done_; }

    /**
     * @return number of frames written to the bus so far, counting repeats when looping
     */
    size_t numFramesSent() const { return frames_sent_; }

    /**
     * @return copy of every frame the CanTransceiver has sent to the bus so far
     */
    std::vector<CAN_FP::CanFrame> outboundFrames() const;

private:
    /**
     * @brief Replay thread body: writes frames to the bus at the pace given by the ReplayConfig
     *
     */
    void replayLoop();

    /**
     * @brief Capture thread body: drains frames the CanTransceiver sends, so its send() never blocks
     *        on a full socket buffer, and records them for outboundFrames()
     *
     */
    void sinkLoop();

    /**
     * @brief Write a single frame to the bus, retrying while the socket buffer is full
     *
     * @param frame frame to write
     * @return whether the frame was written (false if the replay was stopped or the peer is gone)
     */
    bool writeFrame(const CAN_FP::CanFrame & frame);

    int transceiver_fd_;  // Ownership transfers with transceiverFd()
    int bus_fd_;

    std::vector<CAN_REPLAY::TimedFrame> frames_;
    CAN_REPLAY::ReplayConfig            cfg_;

    std::thread       replay_thread_;
    std::thread       sink_thread_;
    std::atomic<bool> stop_replay_{false};
    std::atomic<bool> stop_sink_{false};
    std::atomic<bool> replay_done_{false};

    std::atomic<size_t> frames_sent_{0};

    // Protects outbound_, written by the capture thread and read via outboundFrames()
    mutable std::mutex            outbound_mtx_;
    std::vector<CAN_FP::CanFrame> outbound_;
};
