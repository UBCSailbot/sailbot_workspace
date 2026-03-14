// Copied from original project, ROS dependencies removed
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast.hpp>
#include <memory>

#include "sailbot_db.h"
#include "shared_constants.h"

namespace beast = boost::beast;
namespace http  = beast::http;
namespace bio   = boost::asio;
using tcp       = boost::asio::ip::tcp;

namespace remote_transceiver
{
constexpr int            DEFAULT_NUM_IO_THREADS = 2;
static const std::string PROD_DB_NAME           = "PLACEHOLDER";
static const std::string PROD_HOST              = "0.0.0.0";
constexpr uint16_t       PROD_PORT              = 8081;
static const std::string TESTING_HOST           = "127.0.0.1";
constexpr uint16_t       TESTING_PORT           = 8081;
constexpr int            HTTP_VERSION           = 11;
namespace targets
{
static const std::string ROOT        = "/";
static const std::string SENSORS     = "/sensors";
static const std::string GLOBAL_PATH = "/global-path";
}  // namespace targets
struct MOMsgParams
{
    using Params = struct
    {
        uint64_t    imei_;
        uint32_t    serial_;
        uint16_t    momsn_;
        std::string transmit_time_;
        float       lat_;
        float       lon_;
        uint32_t    cep_;
        std::string data_;
    };
    Params params_;
    explicit MOMsgParams(const std::string & query_string);
    explicit MOMsgParams(Params params) : params_(params) {}
};
class HTTPServer : public std::enable_shared_from_this<HTTPServer>
{
public:
    using LogCallback = std::function<void(const std::string &)>;
    explicit HTTPServer(tcp::socket socket, SailbotDB & db);
    void setLogCallback(LogCallback cb) { log_callback_ = std::move(cb); }
    void doAccept();

private:
    LogCallback                        log_callback_;
    beast::flat_buffer                 buf_{static_cast<std::size_t>(MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES * 2)};
    tcp::socket                        socket_;
    http::request<http::dynamic_body>  req_;
    http::response<http::dynamic_body> res_;
    SailbotDB &                        db_;
    void                               readReq();
    void                               processReq();
    void                               doBadReq();
    void                               doNotFound();
    void                               doGet();
    void                               doPost();
    void                               writeRes();
};
class Listener : public std::enable_shared_from_this<Listener>
{
public:
    using LogCallback = std::function<void(const std::string &)>;
    Listener(bio::io_context & io, tcp::endpoint endpoint, SailbotDB && db);
    void setLogCallback(LogCallback cb) { log_callback_ = std::move(cb); }
    void run();

private:
    LogCallback       log_callback_;
    bio::io_context & io_;
    tcp::acceptor     acceptor_;
    SailbotDB         db_;
};
namespace http_client
{
struct ConnectionInfo
{
    std::string                                             host;
    std::string                                             port;
    std::string                                             target;
    std::tuple<std::string &, std::string &, std::string &> get() { return {host, port, target}; }
};
std::pair<http::status, std::string> get(ConnectionInfo info);
http::status                         post(ConnectionInfo info, std::string content_type, const std::string & body);
http::response<http::dynamic_body>   post_response_body(
    ConnectionInfo info, std::string content_type, const std::string & body);
}  // namespace http_client
}  // namespace remote_transceiver
