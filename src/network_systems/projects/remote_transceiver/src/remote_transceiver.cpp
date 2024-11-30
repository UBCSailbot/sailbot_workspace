#include "remote_transceiver.h"

#include <curl/curl.h>

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
#include <boost/property_tree/json_parser.hpp>  //JSON parser
#include <boost/property_tree/ptree.hpp>
#include <boost/system/error_code.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "cmn_hdrs/shared_constants.h"
#include "global_path.pb.h"
#include "sailbot_db.h"
#include "sensors.pb.h"

using remote_transceiver::HTTPServer;
using remote_transceiver::Listener;
namespace http_client = remote_transceiver::http_client;

// PUBLIC

remote_transceiver::MOMsgParams::MOMsgParams(const std::string & query_string)
{
    static const std::string DATA_KEY = "&data=";

    size_t      data_key_idx  = query_string.find(DATA_KEY);
    std::string iridium_mdata = query_string.substr(0, data_key_idx);
    params_.data_             = query_string.substr(data_key_idx + DATA_KEY.size(), query_string.size());

    // After the HTTP parameters are converted from a string of key-value pairs to an array of strings, keys become
    // the even numbered indices while values become the odd numbered ones. We just need the values.
    constexpr uint8_t IMEI_IDX   = 1;
    constexpr uint8_t SERIAL_IDX = 3;
    constexpr uint8_t MOMSN_IDX  = 5;
    constexpr uint8_t TIME_IDX   = 7;
    constexpr uint8_t LAT_IDX    = 9;
    constexpr uint8_t LON_IDX    = 11;
    constexpr uint8_t CEP_IDX    = 13;

    std::vector<std::string> split_strings(CEP_IDX + 1);  // Minimally sized vector is size CEP_IDX + 1
    boost::algorithm::split(split_strings, iridium_mdata, boost::is_any_of("?=&"));

    params_.imei_          = std::stoi(split_strings[IMEI_IDX]);
    params_.serial_        = std::stoi(split_strings[SERIAL_IDX]);
    params_.momsn_         = std::stoi(split_strings[MOMSN_IDX]);
    params_.transmit_time_ = split_strings[TIME_IDX];
    params_.lat_           = std::stof(split_strings[LAT_IDX]);
    params_.lon_           = std::stof(split_strings[LON_IDX]);
    params_.cep_           = std::stoi(split_strings[CEP_IDX]);
}

HTTPServer::HTTPServer(tcp::socket socket, SailbotDB & db) : socket_(std::move(socket)), db_(db) {}

void HTTPServer::doAccept() { readReq(); }

Listener::Listener(bio::io_context & io, tcp::endpoint endpoint, SailbotDB && db)
: io_(io), acceptor_(bio::make_strand(io)), db_(std::move(db))
{
    beast::error_code ec;

    try {
        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            throw(ec);
        }

        acceptor_.set_option(bio::socket_base::reuse_address(true), ec);
        if (ec) {
            throw(ec);
        }

        acceptor_.bind(endpoint, ec);
        if (ec) {
            throw(ec);
        }

        acceptor_.listen(bio::socket_base::max_listen_connections, ec);
        if (ec) {
            throw(ec);
        }
    } catch (beast::error_code ec) {
        std::cerr << "Error: " << ec.message() << std::endl;
    }
};

void Listener::run()
{
    std::shared_ptr<Listener> self = shared_from_this();
    acceptor_.async_accept(bio::make_strand(io_), [self](beast::error_code e, tcp::socket socket) {
        if (!e) {
            std::make_shared<HTTPServer>(std::move(socket), self->db_)->doAccept();
        } else {
            // Do not throw an error as we can still try to accept new requests
            std::cerr << "Error: " << e.message() << std::endl;
        }
        self->run();
    });
}

// END PUBLIC

// PRIVATE

void HTTPServer::readReq()
{
    std::shared_ptr<HTTPServer> self = shared_from_this();
    req_                             = {};
    http::async_read(socket_, buf_, req_, [self](beast::error_code e, std::size_t /*bytesTransferred*/) {
        if (!e) {
            self->processReq();
        } else {
            std::cerr << "Error: " << e.message() << std::endl;
            std::cerr << self->req_ << std::endl;
        }
    });
}

void HTTPServer::processReq()
{
    res_ = {};
    res_.version(req_.version());
    res_.keep_alive(false);  // Expect very infrequent requests, so disable keep alive

    switch (req_.method()) {
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

// Callback function to write the response data
static size_t WriteCallback(void * contents, size_t size, size_t nmemb, void * userp)
{
    (static_cast<std::string *>(userp))->append(static_cast<char *>(contents), size * nmemb);
    return size * nmemb;
}

// https://docs.rockblock.rock7.com/reference/receiving-mo-messages-via-http-webhook
// IMPORTANT: Have 3 seconds to send HTTP status 200, so do not process data on same thread before responding
void HTTPServer::doPost()
{
    if (req_.target() == remote_transceiver::targets::SENSORS) {
        beast::string_view content_type = req_["content-type"];
        if (content_type == "application/x-www-form-urlencoded") {
            res_.result(http::status::ok);
            std::shared_ptr<HTTPServer> self = shared_from_this();
            // Detach a thread to process the data so that the server can write a response within the 3 seconds allotted
            std::thread post_thread([self]() {
                std::string         query_string = beast::buffers_to_string(self->req_.body().data());
                MOMsgParams::Params params       = MOMsgParams(query_string).params_;
                if (!params.data_.empty()) {
                    Polaris::Sensors       sensors;
                    SailbotDB::RcvdMsgInfo info = {params.lat_, params.lon_, params.cep_, params.transmit_time_};
                    sensors.ParseFromString(params.data_);
                    if (!self->db_.storeNewSensors(sensors, info)) {  //important
                        std::cerr << "Error, failed to store data received from:\n" << info << std::endl;
                    };
                }
            });
            post_thread.detach();
        } else {
            res_.result(http::status::unsupported_media_type);
            res_.set(http::field::content_type, "text/plain");
            beast::ostream(res_.body()) << "Server does not support sensors POST requests of type: " << content_type;
        }
    } else if (req_.target() == remote_transceiver::targets::GLOBAL_PATH) {
        std::shared_ptr<HTTPServer> self     = shared_from_this();
        std::string                 json_str = beast::buffers_to_string(req_.body().data());  //JSON Parsing
        std::stringstream           ss(json_str);
        boost::property_tree::ptree json_tree;
        boost::property_tree::read_json(ss, json_tree);
        std::string         timestamp = json_tree.get<std::string>("timestamp");
        Polaris::GlobalPath global_path;
        int                 num_waypoints = 0;
        for (const auto & waypoint : json_tree.get_child("waypoints")) {
            float               lat             = waypoint.second.get<float>("latitude");
            float               lon             = waypoint.second.get<float>("longitude");
            Polaris::Waypoint * global_waypoint = global_path.add_waypoints();
            global_waypoint->set_longitude(lon);
            global_waypoint->set_latitude(lat);
            num_waypoints++;
        }
        global_path.set_num_waypoints(num_waypoints);
        std::string data;
        //  serialize global path string
        if (!global_path.SerializeToString(&data)) {
            std::cerr << "Failed to Serialized Global Path string" << std::endl;
            std::cerr << global_path.DebugString() << std::endl;
        }

        if (!self->db_.storeNewGlobalPath(global_path, timestamp)) {  //important
            std::cerr << "Error, failed to store data received at:\n" << timestamp << std::endl;
        }

        curl_global_init(CURL_GLOBAL_ALL);

        static constexpr int NUM_CHECK = 20;
        for (int i = 0; i < NUM_CHECK; i++) {
            CURL *      curl;
            CURLcode    res;
            std::string readBuffer;

            curl = curl_easy_init();

            std::string EC        = "B";
            std::string IMEI      = "300434065264590";
            std::string USERNAME  = "myuser";
            std::string test_data = "insertingtest data";

            char * encoded_data = curl_easy_escape(curl, data.c_str(), 0);

            std::string url = "http://localhost:8100/?data=" + std::string(encoded_data) + "&ec=" + EC +
                              "&imei=" + IMEI + "&username=" + USERNAME;

            if (curl != nullptr) {
                curl_free(encoded_data);

                curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

                curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");

                curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);

                curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

                res = curl_easy_perform(curl);

                if (res != CURLE_OK) {
                    std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
                } else {
                    std::stringstream ss(readBuffer);

                    std::string response;
                    std::string error;
                    std::string message;

                    std::getline(ss, response, ',');
                    std::getline(ss, error, ',');
                    std::getline(ss, message, ',');

                    if (!self->db_.storeIridiumResponse(response, error, message, timestamp)) {  //important
                        std::cerr << "Error, failed to store data received at:\n" << timestamp << std::endl;
                    } else {
                        curl_easy_cleanup(curl);
                        break;
                    }
                }
            }
            curl_easy_cleanup(curl);
        }

        curl_global_cleanup();
    } else {
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
    http::async_write(socket_, res_, [self](beast::error_code e, std::size_t /*bytesWritten*/) {
        self->socket_.shutdown(tcp::socket::shutdown_send, e);
        if (e) {
            std::cerr << "Error: " << e.message() << std::endl;
        }
    });
}

// END PRIVATE

std::pair<http::status, std::string> http_client::get(ConnectionInfo info)
{
    bio::io_context io;
    tcp::socket     socket{io};
    tcp::resolver   resolver{io};

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
    if (status == http::status::ok) {
        std::string result = beast::buffers_to_string(res.body().data());
        return {status, result};
    }
    return {status, ""};
}

http::status http_client::post(ConnectionInfo info, std::string content_type, const std::string & body)
{
    bio::io_context io;
    tcp::socket     socket{io};
    tcp::resolver   resolver{io};

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
  ConnectionInfo info, std::string content_type, const std::string & body)
{
    bio::io_context io;
    tcp::socket     socket{io};
    tcp::resolver   resolver{io};

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
