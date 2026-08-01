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
 * @brief How replay timing is derived from the log timestamps
 *
 */
enum class PacingMode {
    REALTIME,      // Follow the log's elapsed time deltas
    SCALED,        // Follow the log's elapsed time deltas divided by rate (ex. 2.0 replays twice as fast)
    FIXED_PERIOD,  // Ignore log timestamps and delay one period before every frame
    FAST           // Send frames back to back as fast as possible
};

/**
 * @brief Decode used to synthesize RUDDER_DATA_FRAME (the e-compass heading) during replay.
 *
 *        Logs recorded before the e-compass published RUDDER_DATA_FRAME (0x050) contain no such
 *        frame at all, so a replayed stack never publishes /rudder. local_pathfinding treats a
 *        missing heading as an inactive input and disables the sail, which makes replay useless
 *        for exercising navigation. Those logs do carry the heading, in an undocumented 16 byte
 *        attitude frame that also appears to hold pitch and roll.
 *
 * @warning This decode is INFERRED from log analysis.
 *          Evidence: the field spans exactly 0..35999 centidegrees, changes by a median of 0.28
 *          degrees between consecutive frames, and tracks GPS derived course over ground.
 *          Treat frames built from it as best-worst case data - it must not be relied on outside
 *          of log replay.
 */
namespace HeadingSynthesis
{
constexpr canid_t SRC_CAN_ID       = 0x204;  // undocumented attitude frame carrying the heading
constexpr size_t  SRC_BYTE_OFF     = 6;      // offset of the heading field within that frame
constexpr size_t  SRC_MIN_DLEN     = 8;      // frame must be this long to contain the field
constexpr int     CENTIDEG_PER_REV = 36000;  // heading values at or above this are invalid
// RudderData stores the heading as degrees * 1000, while the source field is degrees * 100
constexpr uint32_t SRC_TO_RUDDER_SCALE = 10;
}  // namespace HeadingSynthesis

/**
 * @brief Replay behavior configuration
 *
 */
struct ReplayConfig
{
    PacingMode                 mode = PacingMode::REALTIME;
    double                     rate = 1.0;    // SCALED only: log time deltas are divided by this
    std::chrono::milliseconds  period{100};   // FIXED_PERIOD only: delay before every frame
    bool                       loop = false;  // Restart from the top of the log after the last frame
    std::vector<CAN_FP::CanId> allow_ids;     // If non-empty, only these IDs are replayed
    std::vector<CAN_FP::CanId> block_ids;     // These IDs are never replayed
    // REALTIME/SCALED only: cap on the delay between consecutive frames, so gaps in the recording
    // (ex. the logger restarting) pause the replay briefly instead of stalling it for the gap duration
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
     * @brief Add a synthetic RUDDER_DATA_FRAME after every attitude frame that carries a heading,
     *        so that replaying a log without a real e-compass frame still publishes /rudder.
     *        See HeadingSynthesis for the decode and its caveats. Source frames are kept as is,
     *        and synthetic frames reuse their timestamp, so the result stays in chronological order.
     *
     * @param frames frames to augment, in chronological order
     * @return the input frames with synthetic RUDDER_DATA_FRAMEs interleaved
     */
    static std::vector<TimedFrame> synthesizeHeadingFrames(std::vector<TimedFrame> frames);

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
