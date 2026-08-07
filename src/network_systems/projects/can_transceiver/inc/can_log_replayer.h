#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "can_frame_parser.h"

namespace CAN_REPLAY
{

/**
 * @brief A CAN frame paired with the time it was logged
 *
 */
struct TimedFrame
{
    double           t_s;  // Time the frame was logged, in seconds
    CAN_FP::CanFrame frame;
};

/**
 * @brief Replay behavior configuration
 *
 */
struct ReplayConfig
{
    // Log time deltas are divided by this: 1.0 replays in real time, 2.0 at double speed,
    // and <= 0 replays unpaced, sending frames as fast as the transport accepts them
    double                     rate = 1.0;
    bool                       loop = false;
    std::vector<CAN_FP::CanId> allow_ids;  // If non-empty, only these IDs are replayed
    std::vector<CAN_FP::CanId> block_ids;  // These IDs are never replayed
    // Cap on the delay between frames, so gaps in the recording pause the replay briefly
    // instead of stalling it for the whole gap. Ignored when unpaced
    std::chrono::milliseconds max_frame_gap = std::chrono::seconds{1};
};

/**
 * @brief Parses candump-style CSV logs into Linux CAN frames and computes replay pacing
 *        Pure logic - performs no I/O beyond reading the log file, so it is independent of the replay transport
 *
 */
class CanLogReplayer
{
public:
    /**
     * @brief Parse a CSV CAN log into timed frames
     *        Row format: <ISO timestamp>,<elapsed seconds>,<iface>  <hex ID>  [<decimal length>]  <hex bytes>
     *        Ex: 2026-06-06T20:16:32.154049,5645.967,can0  041  [04]  5D 01 40 00
     * @note  Times come from the ISO timestamp column, since combined logs interleave sessions whose
     *        elapsed time bases differ. Rows split mid frame are reassembled; rows with invalid CAN IDs
     *        and otherwise unusable rows are dropped with a warning.
     *
     * @param path path to the CSV log file
     * @return frames in chronological order
     * @throws std::runtime_error if the file cannot be opened
     */
    static std::vector<TimedFrame> parseCsv(const std::string & path);

    /**
     * @brief Apply a ReplayConfig's allow/block ID lists to a set of frames
     *
     * @param frames frames to filter
     * @param cfg    config holding the allow/block lists
     * @return frames that survive the filter, in their original order
     */
    static std::vector<TimedFrame> filter(std::vector<TimedFrame> frames, const ReplayConfig & cfg);

    /**
     * @brief Compute how long to wait before sending frames[idx]
     *
     * @param frames frames being replayed
     * @param idx    index of the frame about to be sent
     * @param cfg    replay configuration
     * @return delay to sleep before sending the frame (never negative)
     */
    static std::chrono::nanoseconds delayBefore(
      const std::vector<TimedFrame> & frames, size_t idx, const ReplayConfig & cfg);
};

}  // namespace CAN_REPLAY
