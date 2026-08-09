#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include "remote_transceiver.h"

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core/bind_handler.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include <boost/beast/core/error.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/core/string_type.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/core/ignore_unused.hpp>
#include <boost/system/error_code.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "sailbot_db.h"
#include "sensors.pb.h"
#include "shared_constants.h"

using remote_transceiver::HTTPServer;
using remote_transceiver::Listener;
namespace http_client = remote_transceiver::http_client;

// RockBLOCK delivers the MO message payload as a hex-encoded string, so it must be converted back to
// raw bytes before it can be parsed as a protobuf. Returns false if the input is not valid hex.
static bool hexToBytes(const std::string &hex, std::string &out)
{
    if (hex.size() % 2 != 0)
    {
        return false;
    }
    out.clear();
    out.reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.size(); i += 2)
    {
        char *end = nullptr;
        const std::string byte_str = hex.substr(i, 2);
        long byte_val = std::strtol(byte_str.c_str(), &end, 16);
        if (end != byte_str.c_str() + 2)
        {
            return false;
        }
        out.push_back(static_cast<char>(byte_val));
    }
    return true;
}

remote_transceiver::MOMsgParams::MOMsgParams(const std::string &query_string)
{
    std::cout << "[DEBUG] Raw query_string: '" << query_string << "'\n";

    std::map<std::string, std::string> fields;
    std::vector<std::string> pairs;
    boost::algorithm::split(pairs, query_string, boost::is_any_of("&"));
    for (const std::string &pair : pairs)
    {
        size_t eq_idx = pair.find('=');
        if (eq_idx == std::string::npos)
        {
            continue;
        }
        fields[pair.substr(0, eq_idx)] = pair.substr(eq_idx + 1);
    }

    std::cout << "[DEBUG] Parsed fields map:";
    for (const auto &[key, value] : fields)
    {
        std::cout << " '" << key << "'='" << value << "'";
    }
    std::cout << "\n";

    auto safe_stoi = [&fields](const char *name) -> int64_t
    {
        auto it = fields.find(name);
        if (it != fields.end() && !it->second.empty())
        {
            try
            {
                return std::stoll(it->second);
            }
            catch (const std::exception &e)
            {
                std::cerr << "[WARN] Invalid number for " << name << ": '" << it->second << "', using -1.\n";
            }
        }
        else
        {
            std::cerr << "[WARN] Missing field for " << name << ", using -1.\n";
        }
        return -1;
    };
    auto safe_stof = [&fields](const char *name) -> float
    {
        auto it = fields.find(name);
        if (it != fields.end() && !it->second.empty())
        {
            try
            {
                return std::stof(it->second);
            }
            catch (const std::exception &e)
            {
                std::cerr << "[WARN] Invalid float for " << name << ": '" << it->second << "', using -1.\n";
            }
        }
        else
        {
            std::cerr << "[WARN] Missing field for " << name << ", using -1.\n";
        }
        return -1.0f;
    };
    auto safe_str = [&fields](const char *name) -> std::string
    {
        auto it = fields.find(name);
        if (it != fields.end() && !it->second.empty())
        {
            return it->second;
        }
        std::cerr << "[WARN] Missing field for " << name << ", using empty string.\n";
        return "";
    };

    params_.imei_ = safe_stoi("imei");
    params_.serial_ = safe_stoi("serial");
    params_.momsn_ = safe_stoi("momsn");
    params_.transmit_time_ = safe_str("transmit_time");
    params_.lat_ = safe_stof("iridium_latitude");
    params_.lon_ = safe_stof("iridium_longitude");
    params_.cep_ = safe_stoi("iridium_cep");
    params_.data_ = safe_str("data");

    std::cout << "[DEBUG] Parsed fields: imei=" << params_.imei_ << ", serial=" << params_.serial_ << ", momsn=" << params_.momsn_ << ", transmit_time='" << params_.transmit_time_ << "', lat=" << params_.lat_ << ", lon=" << params_.lon_ << ", cep=" << params_.cep_ << ", data(len)=" << params_.data_.size() << "\n";
}

HTTPServer::HTTPServer(tcp::socket socket, SailbotDB &db) : socket_(std::move(socket)), db_(db) {}

void HTTPServer::doAccept()
{
    std::cout << "[INFO] doAccept() called, reading request..." << std::endl;
    readReq();
}

Listener::Listener(bio::io_context &io, tcp::endpoint endpoint, SailbotDB &&db)
    : io_(io), acceptor_(bio::make_strand(io)), db_(std::move(db))
{
    std::cout << "[INFO] Server starting on IP: " << endpoint.address().to_string() << ", Port: " << std::to_string(endpoint.port()) << std::endl;
    beast::error_code ec;

    try
    {
        acceptor_.open(endpoint.protocol(), ec);
        if (ec)
        {
            throw(ec);
        }

        acceptor_.set_option(bio::socket_base::reuse_address(true), ec);
        if (ec)
        {
            throw(ec);
        }

        acceptor_.bind(endpoint, ec);
        if (ec)
        {
            throw(ec);
        }

        acceptor_.listen(bio::socket_base::max_listen_connections, ec);
        if (ec)
        {
            throw(ec);
        }
    }
    catch (beast::error_code ec)
    {
        std::cerr << "Error: " << ec.message() << std::endl;
    }
};

void Listener::run()
{
    std::shared_ptr<Listener> self = shared_from_this();
    std::cout << "[INFO] Waiting for incoming connections..." << std::endl;
    acceptor_.async_accept(bio::make_strand(io_), [self, this](beast::error_code e, tcp::socket socket)
    {
        std::cout << "[INFO] Accepting incoming request..." << std::endl;
        if (!e) {
            std::cout << "[INFO] Accepted connection from " << socket.remote_endpoint() << std::endl;
            std::make_shared<HTTPServer>(std::move(socket), self->db_)->doAccept();
        } else {
            std::cerr << "[ERROR] Failed to accept connection: " << e.message() << std::endl;
        }
        self->run();
    });
}

void HTTPServer::readReq()
{
    std::cout << "readReq() called" << std::endl;
    std::shared_ptr<HTTPServer> self = shared_from_this();
    req_ = {};
    http::async_read(socket_, buf_, req_, [self, this](beast::error_code e, std::size_t bytesTransferred)
                     {
        if (!e) {
                std::cout << 
                  "[INFO] Incoming request: Method: " << std::string(self->req_.method_string()) <<
                  ", Target: " << std::string(self->req_.target()) << ", Bytes: " << std::to_string(bytesTransferred) << std::endl;
                std::string headers;
                for (const auto & field : self->req_) {
                    headers += "    " + std::string(field.name_string()) + ": " + std::string(field.value()) + "\n";
                }
                std::cout << "[INFO] Request headers:\n" + headers << std::endl;
                std::cout << "[INFO] Request body: " + beast::buffers_to_string(self->req_.body().data()) << std::endl;
            self->processReq();
        } else {
            std::cerr << "Error: " << e.message() << std::endl;
            std::cerr << self->req_ << std::endl;
        } });
}

void HTTPServer::processReq()
{
    res_ = {};
    res_.version(req_.version());
    res_.keep_alive(false);

    switch (req_.method())
    {
    case http::verb::post:
        doPost();
        break;
    case http::verb::get:
        doGet();
        break;
    default:
        doBadReq();
    }
    writeRes();
}

void HTTPServer::doBadReq()
{
    res_.result(http::status::bad_request);
    res_.set(http::field::content_type, "text/plain");
    beast::ostream(res_.body()) << "Invalid request method: " << req_.method_string();
}

void HTTPServer::doNotFound()
{
    res_.result(http::status::bad_request);
    res_.set(http::field::content_type, "text/plain");
    beast::ostream(res_.body()) << "Not found: " << req_.target();
}

void HTTPServer::doPost()
{
    if (req_.target() == remote_transceiver::targets::SENSORS)
    {
        std::cout << "[INFO] Handling POST to /sensors" << std::endl;
        beast::string_view content_type = req_["content-type"];
        std::cout << "[DEBUG] Content-Type: '" << content_type << "'" << std::endl;
        if (content_type == "application/x-www-form-urlencoded")
        {
            res_.result(http::status::ok);
            std::shared_ptr<HTTPServer> self = shared_from_this();
            std::thread post_thread([self]()
                                    {
                std::string query_string = beast::buffers_to_string(self->req_.body().data());
                std::cout << "[DEBUG] POST body: '" << query_string << "'" << std::endl;
                MOMsgParams::Params params = MOMsgParams(query_string).params_;
                std::cout << "[DEBUG] Parsed params: imei=" << params.imei_ << ", serial=" << params.serial_ << ", momsn=" << params.momsn_ << ", transmit_time='" << params.transmit_time_ << "', lat=" << params.lat_ << ", lon=" << params.lon_ << ", cep=" << params.cep_ << ", data(len)=" << params.data_.size() << std::endl;
                if (!params.data_.empty()) {
                    Polaris::Sensors sensors;
                    SailbotDB::RcvdMsgInfo info = {params.lat_, params.lon_, params.cep_, params.transmit_time_};
                    std::string raw_data;
                    if (!hexToBytes(params.data_, raw_data)) {
                        std::cerr << "[ERROR] data field is not valid hex, skipping: '" << params.data_ << "'" << std::endl;
                        return;
                    }
                    bool parse_ok = sensors.ParseFromString(raw_data);
                    std::cout << "[DEBUG] ParseFromString returned: " << parse_ok << std::endl;
                    if (!parse_ok) {
                        std::cerr << "[ERROR] Failed to parse Sensors protobuf from data." << std::endl;
                        return;
                    }
                    std::cout << "[DEBUG] Storing new sensors in DB..." << std::endl;
                    if (!self->db_.storeNewSensors(sensors, info)) {
                        std::cerr << "[ERROR] Failed to store data received from: lat=" << info.lat_ << ", lon=" << info.lon_ << ", cep=" << info.cep_ << ", time='" << info.timestamp_ << "'" << std::endl;
                    } else {
                        std::cout << "[INFO] Successfully stored new sensors in DB." << std::endl;
                    }
                } else {
                    std::cerr << "[WARN] No data field present in POST body." << std::endl;
                }
            });
            post_thread.detach();
        }
        else
        {
            std::cerr << "[ERROR] Unsupported Content-Type for /sensors: '" << content_type << "'" << std::endl;
            res_.result(http::status::unsupported_media_type);
            res_.set(http::field::content_type, "text/plain");
            beast::ostream(res_.body()) << "Server does not support sensors POST requests of type: " << content_type;
        }
    }
    else
    {
        doNotFound();
    }
}

void HTTPServer::doGet()
{
    res_.result(http::status::ok);
    res_.set(http::field::server, "Sailbot Remote Transceiver");
    res_.set(http::field::content_type, "text/plain");
    beast::ostream(res_.body()) << "PLACEHOLDER\r\n";
}

void HTTPServer::writeRes()
{
    res_.set(http::field::content_length, std::to_string(res_.body().size()));

    std::shared_ptr<HTTPServer> self = shared_from_this();
    http::async_write(socket_, res_, [self](beast::error_code e, std::size_t /*bytesWritten*/)
                      {
        self->socket_.shutdown(tcp::socket::shutdown_send, e);
        if (e) {
            std::cerr << "Error: " << e.message() << std::endl;
        } });
}

std::pair<http::status, std::string> http_client::get(ConnectionInfo info)
{
    bio::io_context io;
    tcp::socket socket{io};
    tcp::resolver resolver{io};

    auto [host, port, target] = info.get();

    tcp::resolver::results_type const results = resolver.resolve(host, port);
    bio::connect(socket, results.begin(), results.end());

    http::request<http::string_body> req{http::verb::get, target, HTTP_VERSION};
    req.set(http::field::host, host);
    req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    http::write(socket, req);

    beast::flat_buffer buf;

    http::response<http::dynamic_body> res;
    http::read(socket, buf, res);

    boost::system::error_code e;
    socket.shutdown(tcp::socket::shutdown_both, e);

    http::status status = res.base().result();
    if (status == http::status::ok)
    {
        std::string result = beast::buffers_to_string(res.body().data());
        return {status, result};
    }
    return {status, ""};
}

http::status http_client::post(ConnectionInfo info, std::string content_type, const std::string &body)
{
    bio::io_context io;
    tcp::socket socket{io};
    tcp::resolver resolver{io};

    auto [host, port, target] = info.get();

    tcp::resolver::results_type const results = resolver.resolve(host, port);
    bio::connect(socket, results.begin(), results.end());

    http::request<http::string_body> req{http::verb::post, target, HTTP_VERSION};
    req.set(http::field::host, host);
    req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    req.set(http::field::content_type, content_type);
    req.set(http::field::content_length, std::to_string(body.size()));
    req.body() = body;
    req.prepare_payload();
    http::write(socket, req);

    beast::flat_buffer buf;

    http::response<http::dynamic_body> res;
    http::read(socket, buf, res);

    boost::system::error_code e;
    socket.shutdown(tcp::socket::shutdown_both, e);

    http::status status = res.base().result();
    return status;
}

http::response<http::dynamic_body> http_client::post_response_body(
    ConnectionInfo info, std::string content_type, const std::string &body)
{
    bio::io_context io;
    tcp::socket socket{io};
    tcp::resolver resolver{io};

    auto [host, port, target] = info.get();

    tcp::resolver::results_type const results = resolver.resolve(host, port);
    bio::connect(socket, results.begin(), results.end());

    http::request<http::string_body> req{http::verb::post, target, HTTP_VERSION};
    req.set(http::field::host, host);
    req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    req.set(http::field::content_type, content_type);
    req.set(http::field::content_length, std::to_string(body.size()));
    req.body() = body;

    req.prepare_payload();
    http::write(socket, req);

    beast::flat_buffer buf;

    http::response<http::dynamic_body> res;
    http::read(socket, buf, res);

    boost::system::error_code e;
    socket.shutdown(tcp::socket::shutdown_both, e);

    return res;
}
