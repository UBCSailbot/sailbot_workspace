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
    double           t_s;    // Elapsed time of the frame in seconds (the log's Elapsed_Time_s column)
    CAN_FP::CanFrame frame;  // Linux CAN representation of the logged frame
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
    bool                       loop = false;  // Restart from the top of the log after the last frame
    std::vector<CAN_FP::CanId> allow_ids;     // If non-empty, only these IDs are replayed
    std::vector<CAN_FP::CanId> block_ids;     // These IDs are never replayed
    // Cap on the delay between consecutive frames, so gaps in the recording (ex. the logger
    // restarting) pause the replay briefly instead of stalling it for the gap duration. Ignored when unpaced
    std::chrono::milliseconds max_frame_gap = std::chrono::seconds{1};
};

/**
 * @brief Parses candump-style CSV logs into Linux CAN frames and computes replay pacing.
 *        Pure logic - performs no I/O beyond reading the log file, so it is independent of the replay transport.
 *
 */
class CanLogReplayer
{
public:
    /**
     * @brief Parse a CSV CAN log into timed frames.
     *        Expected row format: <ISO timestamp>,<elapsed seconds>,<iface>  <hex ID>  [<decimal length>]  <hex bytes>
     *        Ex: 2026-06-06T20:16:32.154049,5645.967,can0  041  [04]  5D 01 40 00
     *
     *        Frame times come from the ISO timestamp column: combined logs interleave sessions whose elapsed
     *        time columns have different bases, which would break pacing. Rows that the log writer split mid
     *        frame across two lines are reassembled. Frames are returned sorted by time. Rows with CAN IDs that
     *        fail CAN_FP::isValidCanId() are dropped, warning once per unique bad ID; other unusable rows are
     *        skipped, warning for the first few only, and a summary is printed at the end.
     *
     * @param path path to the CSV log file
     * @return frames in chronological order
     * @throws std::runtime_error if the file cannot be opened
     */
    static std::vector<TimedFrame> parseCsv(const std::string & path);

    /**
     * @brief Apply the allow/block ID lists from a ReplayConfig to a set of frames
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
