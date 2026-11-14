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
const std::string STATUS_OK    = "OK";
const std::string STATUS_ERROR = "ERROR";
const std::string RSP_READY    = "READY";
const std::string CSQ       = "READY";
const std::string GARBAGE   = "\n";

const std::string CHECK_CONN        = "AT";
const std::string SBD_SESSION       = "AT+SBDIX";  // 5.144
const std::string SBD_SESSION_RING  = "AT+SBDIXA";  // 5.144: same as SBDIX, tells the gateway we are answering a ring
const std::string SBD_STATUS_EXT    = "AT+SBDSX";   // local only (no satellite): buffer flags and ring alert flag
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

// +SBDIX MT status codes
namespace mt_status
{
constexpr uint8_t NO_MESSAGE    = 0;  // mailbox checked, nothing waiting
constexpr uint8_t RECEIVED      = 1;  // one message downloaded into the modem's MT buffer
constexpr uint8_t MAILBOX_ERROR = 2;  // the mailbox could not be checked
}  // namespace mt_status

/**
 * @brief Split an AT response of the form "+NAME:<a>,<b>,..." into its integer fields
 *
 * @param rsp_string  response line
 * @param num_fields  minimum number of fields expected
 * @throws std::invalid_argument if rsp_string is malformed
 */
inline std::vector<int> parseIntFields(const std::string & rsp_string, size_t num_fields)
{
    size_t begin_point = rsp_string.find(':');
    if (begin_point == std::string::npos) {
        throw std::invalid_argument("Malformed AT response: \"" + rsp_string + "\"");
    }
    std::string      data = rsp_string.substr(begin_point + 1);
    std::vector<int> fields;

    size_t start = 0;
    size_t end;
    while ((end = data.find(',', start)) != std::string::npos) {
        fields.push_back(std::stoi(data.substr(start, end - start)));  // stoi skips the spaces some modems add
        start = end + 1;
    }
    fields.push_back(std::stoi(data.substr(start)));

    if (fields.size() < num_fields) {
        throw std::invalid_argument("Malformed AT response: \"" + rsp_string + "\"");
    }
    return fields;
}

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
    uint16_t MT_len_;  // up to MAX_REMOTE_TO_LOCAL_PAYLOAD_SIZE_BYTES (270), too big for a uint8_t
    uint8_t  MT_queued_;

    /**
     * @brief Construct a new Status Response object
     *
     * @param rsp_string string of format "+SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>"
     * @throws std::invalid_argument if rsp_string is malformed
     */
    explicit SBDStatusRsp(const std::string & rsp_string)
    {
        enum { MO_STATUS_INDEX, MOMSN_INDEX, MT_STATUS_INDEX, MTMSN_INDEX, MT_LEN_INDEX, MT_QUEUED_INDEX, NUM_FIELDS };
        std::vector<int> fields = parseIntFields(rsp_string, NUM_FIELDS);

        MO_status_ = fields[MO_STATUS_INDEX];
        MOMSN_     = fields[MOMSN_INDEX];
        MT_status_ = fields[MT_STATUS_INDEX];
        MTMSN_     = fields[MTMSN_INDEX];
        MT_len_    = fields[MT_LEN_INDEX];
        MT_queued_ = fields[MT_QUEUED_INDEX];
    };

    /**
     * @brief Check if the session checked the mailbox. Only then do MT_status_ and MT_queued_ mean anything.
     */
    bool mailboxChecked() const { return MT_status_ != mt_status::MAILBOX_ERROR; }

    /**
     * @brief Check if the session downloaded a message into the modem's MT buffer. It must be read with
     *        AT+SBDRB before the next session, which overwrites the buffer. The gateway has already deleted its copy.
     */
    bool MTReceived() const { return MT_status_ == mt_status::RECEIVED; }

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

/**
 * Struct representing the response to the SBD_STATUS_EXT command. Reading it only talks to the modem, so it is free.
 */
struct SBDStatusExtRsp
{
    bool     MO_flag_;      // a message is in the MO buffer
    uint16_t MOMSN_;
    bool     MT_flag_;      // a message is in the MT buffer
    int32_t  MTMSN_;        // -1 if there is no MT message
    bool     RA_flag_;      // a ring alert arrived and has not been answered yet
    uint16_t msg_waiting_;  // messages queued at the gateway, as of the last SBD session

    /**
     * @brief Construct a new Status Extended Response object
     *
     * @param rsp_string string of format "+SBDSX: <MO flag>, <MOMSN>, <MT flag>, <MTMSN>, <RA flag>, <msg waiting>"
     * @throws std::invalid_argument if rsp_string is malformed
     */
    explicit SBDStatusExtRsp(const std::string & rsp_string)
    {
        enum { MO_FLAG_INDEX, MOMSN_INDEX, MT_FLAG_INDEX, MTMSN_INDEX, RA_FLAG_INDEX, MSG_WAITING_INDEX, NUM_FIELDS };
        std::vector<int> fields = parseIntFields(rsp_string, NUM_FIELDS);

        MO_flag_     = fields[MO_FLAG_INDEX] != 0;
        MOMSN_       = fields[MOMSN_INDEX];
        MT_flag_     = fields[MT_FLAG_INDEX] != 0;
        MTMSN_       = fields[MTMSN_INDEX];
        RA_flag_     = fields[RA_FLAG_INDEX] != 0;
        msg_waiting_ = fields[MSG_WAITING_INDEX];
    }
};

}  // namespace AT
