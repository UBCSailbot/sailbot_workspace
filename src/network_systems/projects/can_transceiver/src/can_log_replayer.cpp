#include "can_log_replayer.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace
{

constexpr int HEX_BASE      = 16;  // candump logs CAN IDs and data bytes in hex
constexpr int DEC_BASE      = 10;  // the bracketed length is decimal
constexpr int NUM_TS_FIELDS = 6;   // year, month, day, hour, minute, seconds

/**
 * @brief Parse a candump-style message column (ex. "can0  041  [04]  5D 01 40 00") into a CAN frame
 *
 * @param msg_str message column of a log row
 * @param frame   output frame, only valid if this function returns true
 * @return whether msg_str was well formed
 */
bool parseCanMessage(const std::string & msg_str, CAN_FP::CanFrame & frame)
{
    std::istringstream iss(msg_str);
    std::string        iface;
    std::string        id_str;
    std::string        len_str;
    if (!(iss >> iface >> id_str >> len_str)) {
        return false;
    }
    if (len_str.size() < 3 || len_str.front() != '[' || len_str.back() != ']') {
        return false;
    }
    try {
        uint64_t id  = std::stoul(id_str, nullptr, HEX_BASE);
        size_t   len = std::stoul(len_str.substr(1, len_str.size() - 2), nullptr, DEC_BASE);
        if (len > CANFD_MAX_DLEN) {
            return false;
        }
        frame        = CAN_FP::CanFrame{};
        frame.can_id = static_cast<canid_t>(id);
        frame.len    = static_cast<uint8_t>(len);
        for (size_t i = 0; i < len; i++) {
            std::string byte_str;
            if (!(iss >> byte_str)) {
                return false;
            }
            uint64_t byte = std::stoul(byte_str, nullptr, HEX_BASE);
            if (byte > 0xFF) {  // NOLINT(readability-magic-numbers)
                return false;
            }
            frame.data[i] = static_cast<uint8_t>(byte);
        }
        // a reassembled row that is still missing bytes must not pass with a frame from a wrong join
        std::string excess;
        if (iss >> excess) {
            return false;
        }
    } catch (const std::exception &) {
        return false;
    }
    return true;
}

/**
 * @brief Parse an ISO timestamp (ex. "2026-06-06T20:16:32.154049") into seconds since the epoch
 *
 * @param ts_str timestamp column of a log row
 * @param t_s    output time in seconds, only valid if this function returns true
 * @return whether ts_str was well formed
 */
bool parseIsoTimestamp(const std::string & ts_str, double & t_s)
{
    int    year;
    int    month;
    int    day;
    int    hour;
    int    minute;
    double sec;
    // NOLINTNEXTLINE(cert-err34-c) - the return value is checked, malformed input is rejected
    if (sscanf(ts_str.c_str(), "%d-%d-%dT%d:%d:%lf", &year, &month, &day, &hour, &minute, &sec) != NUM_TS_FIELDS) {
        return false;
    }
    std::tm tm{};
    tm.tm_year  = year - 1900;  // NOLINT(readability-magic-numbers) - epoch year of std::tm
    tm.tm_mon   = month - 1;
    tm.tm_mday  = day;
    tm.tm_hour  = hour;
    tm.tm_min   = minute;
    tm.tm_sec   = 0;
    time_t base = timegm(&tm);
    if (base == static_cast<time_t>(-1)) {
        return false;
    }
    t_s = static_cast<double>(base) + sec;
    return true;
}

/**
 * @return whether the message column starts a new candump row (as opposed to being the tail of a split row)
 */
bool startsWithIface(const std::string & msg_str)
{
    size_t start = msg_str.find_first_not_of(' ');
    return start != std::string::npos && msg_str.compare(start, 3, "can") == 0;
}

}  // namespace

namespace CAN_REPLAY
{

std::vector<TimedFrame> CanLogReplayer::parseCsv(const std::string & path)
{
    static constexpr size_t MAX_ROW_WARNINGS   = 5;   // avoid flooding the log when a file has many bad rows
    static constexpr size_t MAX_FRAGMENT_JOINS = 3;   // even a 64 byte frame spans at most a few log rows
    static constexpr size_t FRAGMENT_TTL_ROWS  = 50;  // interleaved sessions can separate a fragment and its tail

    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open CAN replay log: " + path);
    }

    std::vector<TimedFrame> frames;
    std::set<canid_t>       invalid_ids;
    size_t                  num_malformed   = 0;
    size_t                  num_reassembled = 0;
    size_t                  num_invalid_id  = 0;

    // Head of a log row that the log writer split mid frame; its tail may arrive several rows later
    // because combined logs interleave rows from multiple recording sessions
    bool        pending_active = false;
    double      pending_t_s    = 0.0;
    std::string pending_text;
    size_t      pending_joins = 0;
    size_t      pending_row   = 0;

    auto warnMalformed = [&num_malformed, &path](size_t line_num) {
        num_malformed++;
        if (num_malformed <= MAX_ROW_WARNINGS) {
            // single << so log frameworks that capture stderr do not interleave or split the message
            std::cerr << ("CAN replay: skipping malformed line " + std::to_string(line_num) + " of " + path + "\n");
        }
    };

    auto emitFrame = [&](double t_s, const CAN_FP::CanFrame & frame) {
        if (!CAN_FP::isValidCanId(frame.can_id)) {
            num_invalid_id++;
            if (invalid_ids.insert(frame.can_id).second) {
                std::ostringstream oss;
                oss << "CAN replay: dropping frames with invalid CAN ID 0x" << std::hex << frame.can_id << "\n";
                std::cerr << oss.str();
            }
            return;
        }
        frames.push_back({t_s, frame});
    };

    std::string line;
    size_t      line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) {
            continue;
        }

        size_t comma_1 = line.find(',');
        size_t comma_2 = (comma_1 == std::string::npos) ? std::string::npos : line.find(',', comma_1 + 1);
        if (comma_2 == std::string::npos) {
            if (line_num > 1) {  // the header row is expected to not parse
                warnMalformed(line_num);
            }
            continue;
        }

        // Frame times come from the ISO timestamp column; the elapsed column is only a fallback because
        // combined logs interleave sessions whose elapsed bases differ, which would break pacing
        double t_s;
        if (!parseIsoTimestamp(line.substr(0, comma_1), t_s)) {
            try {
                t_s = std::stod(line.substr(comma_1 + 1, comma_2 - comma_1 - 1));
            } catch (const std::exception &) {
                if (line_num > 1) {
                    warnMalformed(line_num);
                }
                continue;
            }
        }

        std::string      msg_str = line.substr(comma_2 + 1);
        CAN_FP::CanFrame frame;
        if (parseCanMessage(msg_str, frame)) {
            // note: an in-progress pending fragment is kept - rows of another session may sit between
            // a fragment and its tail
            emitFrame(t_s, frame);
            continue;
        }

        if (pending_active && line_num - pending_row > FRAGMENT_TTL_ROWS) {
            pending_active = false;
            num_malformed++;  // head fragment whose tail never showed up
        }

        if (startsWithIface(msg_str)) {
            // Head of a split row: hold it and wait for its tail
            if (pending_active) {
                num_malformed++;  // previous head never found its tail
            }
            pending_active = true;
            pending_t_s    = t_s;
            pending_text   = msg_str;
            pending_joins  = 0;
            pending_row    = line_num;
            continue;
        }

        // Tail of a split row: the writer split the original line mid token, so join with no separator
        if (!pending_active) {
            warnMalformed(line_num);
            continue;
        }
        pending_text += msg_str;
        pending_joins++;
        if (parseCanMessage(pending_text, frame)) {
            num_reassembled++;
            emitFrame(pending_t_s, frame);
            pending_active = false;
        } else if (pending_joins >= MAX_FRAGMENT_JOINS) {
            num_malformed++;
            pending_active = false;
        }
    }
    if (pending_active) {
        num_malformed++;
    }

    // Combined logs interleave recording sessions, so restore true chronological order for replay
    std::stable_sort(
      frames.begin(), frames.end(), [](const TimedFrame & a, const TimedFrame & b) { return a.t_s < b.t_s; });

    if (num_malformed > 0 || num_reassembled > 0 || num_invalid_id > 0) {
        std::cerr
          << ("CAN replay: parsed " + std::to_string(frames.size()) + " frames from " + path + " (" +
              std::to_string(num_reassembled) + " split rows reassembled, " + std::to_string(num_malformed) +
              " malformed rows skipped, " + std::to_string(num_invalid_id) + " frames dropped across " +
              std::to_string(invalid_ids.size()) + " invalid IDs)\n");
    }
    return frames;
}

std::vector<TimedFrame> CanLogReplayer::filter(std::vector<TimedFrame> frames, const ReplayConfig & cfg)
{
    auto contains = [](const std::vector<CAN_FP::CanId> & ids, canid_t id) {
        return std::find(ids.begin(), ids.end(), static_cast<CAN_FP::CanId>(id)) != ids.end();
    };
    auto rejected = [&cfg, &contains](const TimedFrame & timed_frame) {
        return (!cfg.allow_ids.empty() && !contains(cfg.allow_ids, timed_frame.frame.can_id)) ||
               contains(cfg.block_ids, timed_frame.frame.can_id);
    };
    frames.erase(std::remove_if(frames.begin(), frames.end(), rejected), frames.end());
    return frames;
}

std::vector<TimedFrame> CanLogReplayer::synthesizeHeadingFrames(std::vector<TimedFrame> frames)
{
    std::vector<TimedFrame> out;
    out.reserve(frames.size());

    for (const TimedFrame & timed_frame : frames) {
        out.push_back(timed_frame);

        if (timed_frame.frame.can_id != static_cast<canid_t>(CAN_FP::CanId::RUDDER_DEBUG)) {
            continue;
        }

        try {
            CAN_FP::RudderDebug debug(timed_frame.frame);
            CAN_FP::RudderData  rudder(debug.toRosMsg(), CAN_FP::CanId::RUDDER_DATA_FRAME);
            out.push_back({timed_frame.t_s, rudder.toLinuxCan()});
        } catch (const std::exception &) {
            // a frame that is truncated, carries no heading this cycle, or holds out of bounds
            // state has nothing to publish, so leave it as a source frame only
            continue;
        }
    }
    return out;
}

std::chrono::nanoseconds CanLogReplayer::delayBefore(
  const std::vector<TimedFrame> & frames, size_t idx, const ReplayConfig & cfg)
{
    if (cfg.rate <= 0 || idx == 0 || idx >= frames.size()) {  // a non-positive rate replays unpaced
        return std::chrono::nanoseconds{0};
    }
    double dt_s = (frames[idx].t_s - frames[idx - 1].t_s) / cfg.rate;
    if (dt_s <= 0) {  // guard against out of order or duplicate log timestamps
        return std::chrono::nanoseconds{0};
    }
    // round() rather than duration_cast(): truncation would consistently under-pace the replay
    auto delay = std::chrono::round<std::chrono::nanoseconds>(std::chrono::duration<double>(dt_s));
    // cap the delay so gaps in the recording do not stall the replay
    return std::min(delay, std::chrono::duration_cast<std::chrono::nanoseconds>(cfg.max_frame_gap));
}

}  // namespace CAN_REPLAY
