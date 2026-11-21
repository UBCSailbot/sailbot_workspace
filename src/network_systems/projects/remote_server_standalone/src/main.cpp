// Standalone entry point for the remote server
#include <boost/asio/io_context.hpp>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "remote_transceiver.h"
#include "sailbot_db.h"

// remote_server [--db prod|test]. prod (sailbot_db) is the default, so the deployed server needs no flag.
static std::optional<std::string> dbNameFromArgs(int argc, char ** argv)
{
    const std::vector<std::string> args(argv + 1, argv + argc);
    if (args.empty() || args == std::vector<std::string>{"--db", "prod"}) {
        return remote_transceiver::PROD_DB_NAME;
    }
    if (args == std::vector<std::string>{"--db", "test"}) {
        return remote_transceiver::TEST_DB_NAME;
    }
    return std::nullopt;
}

int main(int argc, char ** argv)
{
    const std::optional<std::string> db_name = dbNameFromArgs(argc, argv);
    if (!db_name) {
        std::cerr << "Usage: remote_server [--db prod|test]" << std::endl;
        return 1;
    }
    try {
        std::string host             = remote_transceiver::TESTING_HOST;
        int         port             = remote_transceiver::TESTING_PORT;
        int         num_threads      = remote_transceiver::DEFAULT_NUM_IO_THREADS;
        std::string mongodb_conn_str = SailbotDB::MONGODB_CONN_STR();
        SailbotDB   sailbot_db(*db_name, mongodb_conn_str);
        if (!sailbot_db.testConnection()) {
            std::cerr << "Failed to connect to database" << std::endl;
            return 1;
        }
        std::cout << "[INFO] Database: " << *db_name << std::endl;
        boost::asio::io_context  io(num_threads);
        boost::asio::ip::address addr = boost::asio::ip::make_address(host);
        // The listener binds the port (and throws if it cannot) before a voyage is started, so an instance that
        // cannot serve never switches the voyage the running server stores real data under
        auto listener = std::make_shared<remote_transceiver::Listener>(
          io, boost::asio::ip::tcp::endpoint{addr, static_cast<uint16_t>(port)}, SailbotDB(*db_name, mongodb_conn_str));
        std::cout << "[INFO] Active voyage: " << sailbot_db.startNewVoyage() << std::endl;
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
