#pragma once

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast.hpp>
#include <memory>

#include "cmn_hdrs/shared_constants.h"
#include "sailbot_db.h"

namespace beast = boost::beast;
namespace http  = beast::http;
namespace bio   = boost::asio;
using tcp       = boost::asio::ip::tcp;

namespace remote_transceiver
{
// Default number of HTTP requests that can be accepted in parallel
constexpr int DEFAULT_NUM_IO_THREADS = 2;

// Production constants are all placheholders
static const std::string PROD_DB_NAME = "PLACEHOLDER";
static const std::string PROD_HOST    = "127.0.0.1";
constexpr uint16_t       PROD_PORT    = 8081;

// TESTING constants should match the webhook server endpoint info found in sailbot_workspace/scripts/run_virtual_iridium.sh
static const std::string TESTING_HOST = "127.0.0.1";
constexpr uint16_t       TESTING_PORT = 8081;

constexpr int HTTP_VERSION = 11;  // HTTP v1.1
namespace targets
{
static const std::string ROOT        = "/";
static const std::string SENSORS     = "/sensors";
static const std::string GLOBAL_PATH = "/global-path";
}  // namespace targets

/**
 * Struct representing the HTTP POST request sent to the server from Iridium
 * https://docs.rockblock.rock7.com/reference/receiving-mo-messages-via-http-webhook
 *
 */
struct MOMsgParams
{
    using Params = struct
    {
        uint64_t    imei_;           // Rockblock IMEI
        uint32_t    serial_;         // Rockblock serial #. Don't know the max size
        uint16_t    momsn_;          // # msgs sent from the Rockblock [0, 65535]
        std::string transmit_time_;  // UTC date and time. Ex: "21-10-31 10:41:50"
        float       lat_;            // transmitted from this latitude
        float       lon_;            // transmitted from this longitude
        uint32_t    cep_;            // estimate of the accuracy (in km) of the reported lat_ lon_ fields
        std::string data_;           // hex-encoded sensor data from the local_transceiver
    };
    Params params_;

    /**
     * @brief Construct a new MOMsg object from an HTTP query string
     *
     * @param query_string example:
     *      imei=1234&serial=5678&momsn=9123&transmit_time=21-10-31 10:41:50&iridium_latitude=12.34&iridium_longitude=56.78&iridium_cep=2&data=A1B2C3
     */
    explicit MOMsgParams(const std::string & query_string);

    /**
     * @brief Construct a new MOMsgParams object from Params struct.
     *
     * @param params
     */
    explicit MOMsgParams(Params params) : params_(params) {}
};

/**
 * HTTPServer class to handle all HTTP requests directed to the remote transceiver
 *
 */
class HTTPServer : public std::enable_shared_from_this<HTTPServer>
{
public:
    /**
     * @brief Construct a new HTTPServer object
     *
     * @param socket TCP socket
     * @param db     SailbotDB instance
     */
    explicit HTTPServer(tcp::socket socket, SailbotDB & db);

    /**
     * @brief Process an accepted HTTP request
     *
     */
    void doAccept();

private:
    // Buffer to store request data. Double MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES to have room for Iridium metadata
    beast::flat_buffer                 buf_{static_cast<std::size_t>(MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES * 2)};
    tcp::socket                        socket_;  // Socket the server is attached to
    http::request<http::dynamic_body>  req_;     // Current request the server is processing
    http::response<http::dynamic_body> res_;     // Server response
    SailbotDB &                        db_;      // SailbotDB instance

    /**
     * @brief After accepting a request, read its contents into buf_
     *
     */
    void readReq();

    /**
     * @brief After reading a request, determine how to handle it and issue the relevant response.
     *
     */
    void processReq();

    /**
     * @brief Set res_ to indicate an unsupported HTTP request type
     *
     */
    void doBadReq();

    /**
     * @brief Set res_ to indicate that the request target was not found
     *
     */
    void doNotFound();

    /**
     * @brief Respond to a GET request
     *
     */
    void doGet();

    /**
     * @brief Respond to a POST request
     *
     */
    void doPost();

    /**
     * @brief Send out the res_
     *
     */
    void writeRes();
};

/**
 * Listener class to listen for and accept HTTP requests over TCP
 *
 */
class Listener : public std::enable_shared_from_this<Listener>
{
public:
    /**
     * @brief Create a new Listener
     *
     * @param io       reference to io_context
     * @param acceptor tcp::acceptor configured with desired host and port
     * @param db       SailbotDB instance - the Listener requires that it takes ownership of the db
     */
    Listener(bio::io_context & io, tcp::endpoint endpoint, SailbotDB && db);

    /**
     * @brief Run the Listener
     *
     */
    void run();

private:
    bio::io_context & io_;        // io_context used by this Listener
    tcp::acceptor     acceptor_;  //tcp::acceptor configured with desired host, port, and target
    SailbotDB         db_;        // SailbotDB attached to this Listener
};

namespace http_client
{
/**
 * Struct comprised of common fields needed to initiate HTTP sessiosn with boost::beast
 *
 */
struct ConnectionInfo
{
    std::string host;    // Ex. TESTING_HOST
    std::string port;    // Ex. TESTING_PORT
    std::string target;  // See targets namespace

    /**
     * @brief Convenience function to access the contents of the struct
     *
     * @return tuple of references to ConnectionInfo fields
     */
    std::tuple<std::string &, std::string &, std::string &> get() { return {host, port, target}; }
};

/**
 * @brief Send an HTTP GET request
 *
 * @param info ConnectionInfo configuration
 * @return http::status of the response and
 *              Response body if status is OK
 *              Nothing otherwise
 */
std::pair<http::status, std::string> get(ConnectionInfo info);

/**
 * @brief Send an HTTP POST request
 *
 * @param info          ConnectionInfo configuration
 * @param content_type  what kind of content is being posted (ex. application/x-www-form-urlencoded)
 * @param body          Content to POST
 * @return http::status of the response
 */
http::status post(ConnectionInfo info, std::string content_type, const std::string & body);

/**
 * @brief Send an HTTP POST request
 *
 * @param info          ConnectionInfo configuration
 * @param content_type  what kind of content is being posted (ex. application/x-www-form-urlencoded)
 * @param body          Content to POST
 * @return http::response of the response
 */
http::response<http::dynamic_body> post_response_body(
  ConnectionInfo info, std::string content_type, const std::string & body);
}  // namespace http_client

}  // namespace remote_transceiver
