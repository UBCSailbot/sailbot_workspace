// Standalone entry point for the remote server
#include <boost/asio/io_context.hpp>
#include <iostream>
#include <thread>

#include "remote_transceiver.h"
#include "sailbot_db.h"

int main(int argc, char ** argv)
{
    try {
        std::string db_name          = "test";
        std::string host             = remote_transceiver::TESTING_HOST;
        int         port             = remote_transceiver::TESTING_PORT;
        int         num_threads      = remote_transceiver::DEFAULT_NUM_IO_THREADS;
        std::string mongodb_conn_str = SailbotDB::MONGODB_CONN_STR();
        SailbotDB   sailbot_db(db_name, mongodb_conn_str);
        if (!sailbot_db.testConnection()) {
            std::cerr << "Failed to connect to database" << std::endl;
            return 1;
        }
        boost::asio::io_context  io(num_threads);
        boost::asio::ip::address addr     = boost::asio::ip::make_address(host);
        auto                     listener = std::make_shared<remote_transceiver::Listener>(
          io, boost::asio::ip::tcp::endpoint{addr, static_cast<uint16_t>(port)}, std::move(sailbot_db));
        listener->setLogCallback([](const std::string & msg) { std::cout << msg << std::endl; });
        listener->run();
        std::vector<std::thread> io_threads(num_threads);
        for (auto & t : io_threads) {
            t = std::thread([&io]() { io.run(); });
        }
        for (auto & t : io_threads) {
            t.join();
        }
    } catch (const std::exception & e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
