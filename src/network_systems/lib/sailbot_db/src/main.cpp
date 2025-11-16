#include <boost/program_options.hpp>

#include "util_db.h"

namespace po = boost::program_options;

enum class CLIOpt { Help, Clear, Populate, Seed, DumpSensors, DumpGlobalPath, DBName };

std::string to_string(CLIOpt c)
{
    switch (c) {
        case CLIOpt::Help:
            return "help";
        case CLIOpt::Clear:
            return "clear";
        case CLIOpt::Populate:
            return "populate";
        case CLIOpt::Seed:
            return "seed";
        case CLIOpt::DumpSensors:
            return "dump-sensors";
        case CLIOpt::DumpGlobalPath:
            return "dump-global-path";
        case CLIOpt::DBName:
            return "db-name";
    }
};

const std::map<CLIOpt, std::string> CLIOptDesc{
  {CLIOpt::Help, "Help message"},
  {CLIOpt::Clear, "Clear the contents of a database collection"},
  {CLIOpt::Populate, "Populate a database collection with random data"},
  {CLIOpt::Seed, "(Optional) Unsigned integer random seed to generate random data used to populate db collection"},
  {CLIOpt::DumpSensors, "Dump latest sensor data in the database collection"},
  {CLIOpt::DumpGlobalPath, "Dump latest Global Path stored in the database collection"},
  {CLIOpt::DBName, "Name of db collection to target"},
};

int main(int argc, char ** argv)
{
    try {
        // Formatting is weird, see: https://www.boost.org/doc/libs/1_63_0/doc/html/program_options/tutorial.html
        po::options_description o_desc("COMMAND(s)");
        // The ",h" allows for a shortened -h flag
        o_desc.add_options()((to_string(CLIOpt::Help) + ",h").c_str(), CLIOptDesc.at(CLIOpt::Help).c_str());
        o_desc.add_options()(to_string(CLIOpt::Clear).c_str(), CLIOptDesc.at(CLIOpt::Clear).c_str());
        o_desc.add_options()(to_string(CLIOpt::Populate).c_str(), CLIOptDesc.at(CLIOpt::Populate).c_str());
        o_desc.add_options()(
          to_string(CLIOpt::Seed).c_str(), po::value<uint32_t>(), CLIOptDesc.at(CLIOpt::Seed).c_str());
        o_desc.add_options()(to_string(CLIOpt::DumpSensors).c_str(), CLIOptDesc.at(CLIOpt::DumpSensors).c_str());
        o_desc.add_options()(to_string(CLIOpt::DumpGlobalPath).c_str(), CLIOptDesc.at(CLIOpt::DumpGlobalPath).c_str());
        o_desc.add_options()(
          to_string(CLIOpt::DBName).c_str(), po::value<std::string>(), CLIOptDesc.at(CLIOpt::DBName).c_str());

        // Make DBName a positional argument so we don't have to specify --db-name
        po::positional_options_description po_desc;
        po_desc.add(to_string(CLIOpt::DBName).c_str(), -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(o_desc).positional(po_desc).run(), vm);
        po::notify(vm);

        const std::string usage_instructions = [&o_desc]() {
            std::stringstream ss;
            ss << "Usage: sailbot_db DB-NAME [COMMAND]\n\n"
               // Need to separately print that DB-NAME is a positional argument
               << "DB-NAME: " << CLIOptDesc.at(CLIOpt::DBName) << "\n\n"
               << o_desc << std::endl;
            return ss.str();
        }();

        if (vm.count(to_string(CLIOpt::Help)) != 0) {
            std::cout << usage_instructions << std::endl;
            return 0;
        }

        if (vm.count(to_string(CLIOpt::DBName)) == 0) {
            std::cerr << usage_instructions << std::endl;
            return -1;
        }

        uint32_t seed;
        if (vm.count(to_string(CLIOpt::Seed)) != 0) {
            seed = vm[to_string(CLIOpt::Seed)].as<uint32_t>();
        } else {
            seed = std::random_device()();  // initialize seed with random device value
        }
        std::mt19937 mt(seed);

        std::string db_name = vm[to_string(CLIOpt::DBName)].as<std::string>();
        UtilDB      db(db_name, SailbotDB::MONGODB_CONN_STR(), std::make_shared<std::mt19937>(mt));

        if (!db.testConnection()) {
            std::cerr << "Failed to establish connection to DB \"" << db_name << "\"" << std::endl;
            return -1;
        }

        if (vm.count(to_string(CLIOpt::Clear)) != 0) {
            db.cleanDB();
        }

        if (vm.count(to_string(CLIOpt::Populate)) != 0) {
            std::cout << "Populating random sensors with seed: " << seed << std::endl;
            auto [rand_sensors, info] = db.genRandData(UtilDB::getTimestamp());
            db.storeNewSensors(rand_sensors, info);
            // TODO(hsn200406,vaibhavambastha): Add code to store global path
        }

        if (vm.count(to_string(CLIOpt::DumpSensors)) != 0) {
            utils::FailTracker t;
            auto [sensors_vec, timestamp_vec] = db.dumpSensors(t, 1);
            std::cout << "Latest sensors:\n\n" << sensors_vec.at(sensors_vec.size() - 1).DebugString() << std::endl;
            std::cout << "Timestamp: " << timestamp_vec.at(sensors_vec.size() - 1) << std::endl;
        }

        if (vm.count(to_string(CLIOpt::DumpGlobalPath)) != 0) {
            std::cerr << "Dump global path not implemented!" << std::endl;
            // TODO(hsn200406,vaibhavambastha): Add code to dump global path
        }

        return 0;
    } catch (std::exception & e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }
}
