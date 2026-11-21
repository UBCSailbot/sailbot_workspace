#pragma once

// Full command set: https://cdn-shop.adafruit.com/product-files/4521/4521-AT%20command.pdf
// Section numbers in this header file refer to this document

#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace AT
{

const std::string DELIMITER = "\r\n";
const std::string STATUS_OK = "OK";
const std::string RSP_READY = "READY";
const std::string CSQ       = "READY";
const std::string GARBAGE   = "\n";

const std::string CHECK_CONN        = "AT";
const std::string SBD_SESSION       = "AT+SBDIX";  // 5.144
const std::string CLEAR_MO_BUFFER   = "AT+SBDD0";  // 5.142: clear the mobile-originated buffer
const std::string DSBL_CTRLFLOW     = "AT&K0";
const std::string DNLD_TO_QUEUE     = "AT+SBDRB";
const std::string CHECK_SIG_QUALITY = "AT+CSQ";

namespace write_bin  // 5.154
{
const std::string CMD = "AT+SBDWB=";

namespace rsp
{
const std::string SUCCESS      = "0";
const std::string TIMEOUT      = "1";
const std::string BAD_CHECKSUM = "2";
const std::string WRONG_SIZE   = "3";
}  // namespace rsp
}  // namespace write_bin

namespace signal_quality
{
const int NOSIGNAL  = 0;
const int POOR      = 1;
const int MARGINAL  = 2;
const int GOOD      = 3;
const int VERYGOOD  = 4;
const int EXCELLENT = 5;
}  // namespace signal_quality

// +SBDIX MO status codes that need special handling. Codes not listed here are either success (0-4) or
// transient/reserved failures that are worth retrying.
// Reference: https://docs.groundcontrol.com/iot/rockblock/user-manual/at-commands
namespace mo_status
{
constexpr uint8_t GSS_QUEUE_FULL       = 11;
constexpr uint8_t TOO_MANY_SEGMENTS    = 12;
constexpr uint8_t INVALID_SEGMENT_SIZE = 14;
constexpr uint8_t ACCESS_DENIED        = 15;
constexpr uint8_t ISU_LOCKED           = 16;
constexpr uint8_t ANTENNA_FAULT        = 33;
constexpr uint8_t RADIO_DISABLED       = 34;
constexpr uint8_t REGISTRATION_WAIT    = 36;  // must wait 3 minutes since last registration
constexpr uint8_t SBD_DISABLED         = 37;  // SBD service temporarily disabled
constexpr uint8_t TRAFFIC_MANAGEMENT   = 38;
constexpr uint8_t BAND_VIOLATION       = 64;
constexpr uint8_t PLL_LOCK_FAILURE     = 65;
}  // namespace mo_status

/**
 * @brief How to react to the MO status of an SBD session
 */
enum class MOResult {
    SUCCESS,      // message delivered
    RETRY_SOON,   // transient failure (signal drop, no network, busy), retrying is free on RockBLOCK
    RETRY_LATER,  // gateway asked us to wait minutes, an immediate retry cannot succeed
    FATAL         // retrying cannot fix this (account, hardware, or message problem)
};

/**
 * @brief Simple Line struct to help enforce DRY when dealing with strings while performing reads and writes
 *
 */
struct Line
{
    /**
     * @param str valid AT command or response string
     */
    inline explicit Line(const std::string & str)
    : str_((str == AT::DELIMITER || str == "\n" || str == "\r") ? str : (str + "\r"))
    {
    }
    // In most cases, str_ will just be the input str to the constructor + "\r"
    // AT::DELIMITER, \n, and \r are exceptions, and remain the same
    const std::string str_;
};

/**
 * Struct representing the response to the CHECK_STATUS command
 * 5.144
 */
struct SBDStatusRsp
{
    static constexpr uint8_t MO_SUCCESS_START = 0;
    static constexpr uint8_t MO_SUCCESS_END   = 4;

    uint8_t  MO_status_;  // indicates if MO message is transferred successfully [0, 4]
    uint16_t MOMSN_;
    uint8_t  MT_status_;
    uint16_t MTMSN_;
    uint8_t  MT_len_;
    uint8_t  MT_queued_;

    /**
     * @brief Construct a new Status Response object
     *
     * @param rsp_string string of format "+SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>"
     * @throws std::invalid_argument if rsp_string is malformed
     */
    explicit SBDStatusRsp(const std::string & rsp_string)
    {
        size_t begin_point = rsp_string.find(':');
        if (begin_point == std::string::npos) {
            throw std::invalid_argument("Malformed SBDIX response: \"" + rsp_string + "\"");
        }
        std::string              data = rsp_string.substr(begin_point + 1);
        std::vector<std::string> tokens;

        size_t start = 0;
        size_t end;
        while ((end = data.find(',', start)) != std::string::npos) {
            tokens.push_back(data.substr(start, end - start));
            start = end + 1;
        }
        tokens.push_back(data.substr(start));

        // assign index numbers
        enum { MO_STATUS_INDEX, MOMSN_INDEX, MT_STATUS_INDEX, MTMSN_INDEX, MT_LEN_INDEX, MT_QUEUED_INDEX, NUM_FIELDS };

        if (tokens.size() < NUM_FIELDS) {
            throw std::invalid_argument("Malformed SBDIX response: \"" + rsp_string + "\"");
        }

        MO_status_ = std::stoi(tokens[MO_STATUS_INDEX]);
        MOMSN_     = std::stoi(tokens[MOMSN_INDEX]);
        MT_status_ = std::stoi(tokens[MT_STATUS_INDEX]);
        MTMSN_     = std::stoi(tokens[MTMSN_INDEX]);
        MT_len_    = std::stoi(tokens[MT_LEN_INDEX]);
        MT_queued_ = std::stoi(tokens[MT_QUEUED_INDEX]);
    };

    /**
     * @brief Check if last Mobile Originated (i.e. transmitted sensors) transaction was successful
     *
     * @return true  on success
     * @return false on failure
     */
    bool MOSuccess() const { return MO_status_ <= MO_SUCCESS_END; }

    /**
     * @brief Classify the MO status to decide whether the session should be retried
     */
    MOResult classifyMO() const
    {
        if (MOSuccess()) {
            return MOResult::SUCCESS;
        }
        switch (MO_status_) {
            case mo_status::GSS_QUEUE_FULL:
            case mo_status::TOO_MANY_SEGMENTS:
            case mo_status::INVALID_SEGMENT_SIZE:
            case mo_status::ACCESS_DENIED:
            case mo_status::ISU_LOCKED:
            case mo_status::ANTENNA_FAULT:
            case mo_status::RADIO_DISABLED:
            case mo_status::BAND_VIOLATION:
            case mo_status::PLL_LOCK_FAILURE:
                return MOResult::FATAL;
            case mo_status::REGISTRATION_WAIT:
            case mo_status::SBD_DISABLED:
            case mo_status::TRAFFIC_MANAGEMENT:
                return MOResult::RETRY_LATER;
            default:
                return MOResult::RETRY_SOON;
        }
    }
};

}  // namespace AT
