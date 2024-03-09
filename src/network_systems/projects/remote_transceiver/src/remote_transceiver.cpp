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
#include <iostream>
#include <memory>
#include <string>

#include "cmn_hdrs/shared_constants.h"
#include "sailbot_db.h"
#include "sensors.pb.h"

using remote_transceiver::HTTPServer;
using remote_transceiver::Listener;
namespace http_client = remote_transceiver::http_client;

// PUBLIC

remote_transceiver::MOMsgParams::MOMsgParams(const std::string & query_string)
{
    static const std::string DATA_KEY =
      "&data=";  // substring that signifies the beginning of payload data and end of iridium header

    size_t data_key_idx = query_string.find(DATA_KEY);  // find first index of where &data= is, returns the index of &
    std::string iridium_mdata = query_string.substr(0, data_key_idx);  // obtains the iridium header as a string
    params_.data_ =
      query_string.substr(data_key_idx + DATA_KEY.size(), query_string.size());  // obtains the actual payload data

    // After the HTTP parameters are converted from a string of key-value pairs to an array of strings, keys become
    // the even numbered indices while values become the odd numbered ones. We just need the values.
    constexpr uint8_t IMEI_IDX   = 1;
    constexpr uint8_t SERIAL_IDX = 3;
    constexpr uint8_t MOMSN_IDX  = 5;
    constexpr uint8_t TIME_IDX   = 7;
    constexpr uint8_t LAT_IDX    = 9;
    constexpr uint8_t LON_IDX    = 11;
    constexpr uint8_t CEP_IDX    = 13;

    // create an array of 14 strings to hold the iridium header as [[info], [value] ...]
    // since there are 7 pieces of information in the header, each will be parsed as one [info], [value] pair we
    // make an array of size 14 to hold the header
    std::vector<std::string> split_strings(CEP_IDX + 1);  // Minimally sized vector is size CEP_IDX + 1
    // take the iridium header, and parse the [[key], [values]] with delimiters =, &, and ?
    // similar to parsing over the array and when it encounters one of delimiters, stores parsed so far as new entry in array
    boost::algorithm::split(split_strings, iridium_mdata, boost::is_any_of("?=&"));

    // access the array and obtain the values of the header and set them in the internal struct
    params_.imei_          = std::stoi(split_strings[IMEI_IDX]);
    params_.serial_        = std::stoi(split_strings[SERIAL_IDX]);
    params_.momsn_         = std::stoi(split_strings[MOMSN_IDX]);
    params_.transmit_time_ = split_strings[TIME_IDX];
    params_.lat_           = std::stof(split_strings[LAT_IDX]);
    params_.lon_           = std::stof(split_strings[LON_IDX]);
    params_.cep_           = std::stoi(split_strings[CEP_IDX]);
}

// constructor for the HTTP server that takes its own socket and refernce to public db and saves them
HTTPServer::HTTPServer(tcp::socket socket, SailbotDB & db) : socket_(std::move(socket)), db_(db) {}

// accepts its connection and calls private readReq function
void HTTPServer::doAccept() { readReq(); }

// listener constructor that opens a tcp endpoint and listens to it
Listener::Listener(bio::io_context & io, tcp::endpoint endpoint, SailbotDB && db)
: io_(io), acceptor_(bio::make_strand(io)), db_(std::move(db))
{
    beast::error_code ec;

    try {
        // open a socket given the endpoint
        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            throw(ec);
        }
        // use the same address for the acceptor
        acceptor_.set_option(bio::socket_base::reuse_address(true), ec);
        if (ec) {
            throw(ec);
        }
        // bind to the socket
        acceptor_.bind(endpoint, ec);
        if (ec) {
            throw(ec);
        }

        // listen to the socket for incoming connections
        acceptor_.listen(bio::socket_base::max_listen_connections, ec);
        if (ec) {
            throw(ec);
        }
    } catch (beast::error_code ec) {
        std::cerr << "Error: " << ec.message() << std::endl;
    }
};

// endless run function that creates an HTTP server and attaches it to the sailbot_db instance and its own private socket then instructs the new server to
// accept the new connection request -> want one strand per listener
void Listener::run()
{
    // obtain pointer to listener
    std::shared_ptr<Listener> self = shared_from_this();
    // asynchronously wait for new connection and accept on a strand, calls the lambda function on new message
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

// reads the connection request and call processing function
void HTTPServer::readReq()
{
    // obtains a pointer to the HTTPserver associated with caller so it can be accessed in the callback function
    std::shared_ptr<HTTPServer> self = shared_from_this();
    // asynchronously read from the socket which contains our request into req_ and process it
    req_ = {};
    http::async_read(socket_, buf_, req_, [self](beast::error_code e, std::size_t /*bytesTransferred*/) {
        if (!e) {
            // call processReq
            self->processReq();
        } else {
            std::cerr << "Error: " << e.message() << std::endl;
            std::cerr << self->req_ << std::endl;
        }
    });
}

// given a request to the server, choose how to handle it
void HTTPServer::processReq()
{
    // set result version cause its the same
    res_ = {};
    res_.version(req_.version());
    // disconnect after receiving post
    res_.keep_alive(false);  // Expect very infrequent requests, so disable keep alive

    // case statement for request
    switch (req_.method()) {
        case http::verb::post:  // if receive post request, handle it downstream or upstream
            doPost();
            break;
        case http::verb::get:  // if receive get request, handle it
            doGet();
            break;
        default:
            doBadReq();  // if request is not handled, return bad request
    }

    // send the response
    writeRes();
}

// handler if we are given an unhandled request type
void HTTPServer::doBadReq()
{
    // set the response type to bad request
    res_.result(http::status::bad_request);
    // set the content type of the response to plain text
    res_.set(http::field::content_type, "text/plain");
    // write invalid request method to the data of response
    beast::ostream(res_.body()) << "Invalid request method: " << req_.method_string();
}

// used in doPost in the case where we are trying to post to an un-managed resource
void HTTPServer::doNotFound()
{
    // set the request response to a bad request
    res_.result(http::status::bad_request);
    // set the response data to empty text
    res_.set(http::field::content_type, "text/plain");
    beast::ostream(res_.body()) << "Not found: " << req_.target();
}

// https://docs.rockblock.rock7.com/reference/receiving-mo-messages-via-http-webhook
// IMPORTANT: Have 3 seconds to send HTTP status 200, so do not process data on same thread before responding
// used when we get a Post request, handle each request differently based on the type of data received
void HTTPServer::doPost()
{
    // from local to global with data on boat status -> upstream
    if (req_.target() == remote_transceiver::targets::SENSORS) {
        // obtain the content type
        beast::string_view content_type = req_["content-type"];
        // since we only support the application/x-www-form-urlencoded type (from iridium), check for this
        if (content_type == "application/x-www-form-urlencoded") {
            res_.result(http::status::ok);                          // set result status to ok
            std::shared_ptr<HTTPServer> self = shared_from_this();  // obtain reference to this instance of HTTP server
            // Detach a thread to process the data so that the server can write a response within the 3 seconds allotted
            std::thread post_thread([self]() {  // create a thread and start executing the function given here
                // parse data of post to a string
                std::string query_string = beast::buffers_to_string(self->req_.body().data());
                // create a sensor parameters object which parses the string into a C++ data representation
                MOMsgParams::Params params = MOMsgParams(query_string).params_;
                if (!params.data_.empty()) {   // if our params has data in it
                    Polaris::Sensors sensors;  // create new sensors object
                    // obtain information about iridium header which contains lat, lon, cep_ and transmit_time_ and put it
                    // into a sailbot db message info object
                    SailbotDB::RcvdMsgInfo info = {params.lat_, params.lon_, params.cep_, params.transmit_time_};

                    // create polaris sensors object from data_string sent from iridium
                    sensors.ParseFromString(params.data_);
                    // store sensors object into data base or print out info value if failed
                    if (!self->db_.storeNewSensors(sensors, info)) {
                        std::cerr << "Error, failed to store data received from:\n" << info << std::endl;
                    };
                }
            });
            post_thread.detach();  // detach this thread and make it execute in parallel on its own
        } else {
            // send a response that the data is unsupported
            res_.result(http::status::unsupported_media_type);
            res_.set(http::field::content_type, "text/plain");
            beast::ostream(res_.body()) << "Server does not support sensors POST requests of type: " << content_type;
        }
        // from global to local with global waypoints -> downstream to boat
    } else if (req_.target() == remote_transceiver::targets::GLOBAL_PATH) {
        // TODO(): Allow POST global path
        res_.result(http::status::not_implemented);
    } else {
        doNotFound();
    }
}

// since we currently have no need for get requests, simply return an empty placeholder
void HTTPServer::doGet()
{
    // set response status to ok
    res_.result(http::status::ok);
    // set server field to sailbot remote transciever
    res_.set(http::field::server, "Sailbot Remote Transceiver");
    // set content type to plain text
    res_.set(http::field::content_type, "text/plain");
    // put a placeholder string into the body of the response
    beast::ostream(res_.body()) << "PLACEHOLDER\r\n";
}

// once we have been given our formed HTTP response, we write the response to output socket
void HTTPServer::writeRes()
{
    // set content_length field
    res_.set(http::field::content_length, std::to_string(res_.body().size()));

    // obtain pointer to ourself
    std::shared_ptr<HTTPServer> self = shared_from_this();
    // asynchronously write the result to the socket and run the given callback function once writing is complete
    http::async_write(socket_, res_, [self](beast::error_code e, std::size_t /*bytesWritten*/) {
        // the callback function shutsdown the socket since this is a non-persistent connection
        self->socket_.shutdown(tcp::socket::shutdown_send, e);
        if (e) {
            std::cerr << "Error: " << e.message() << std::endl;
        }
    });
}

// END PRIVATE

// sends a get request to the test server used in testing
std::pair<http::status, std::string> http_client::get(ConnectionInfo info)
{
    // create io, socket and TCP resolver
    bio::io_context io;
    tcp::socket     socket{io};
    tcp::resolver   resolver{io};

    // obtain information to the test server
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

// sends a post request to test server used in testing
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
