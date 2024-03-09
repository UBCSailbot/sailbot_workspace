#include "parse_yaml.h"

std::string readFile(const char * file_path)
{
    std::ifstream in_file(file_path, std::ios::in | std::ios::binary);

    if (!in_file) {
        std::cerr << "Could not open file: " << file_path << std::endl;
        throw std::ifstream::failure("ERROR: ifstream could not open file");
    }

    std::stringstream file_buffer;
    file_buffer << in_file.rdbuf();

    return file_buffer.str();
}

testType getTestTypeFromStr(std::string test_type_str)
{
    if (test_type_str == "ROS") {
        return ROS;
    } else if (test_type_str == "CAN") {
        return CAN;
    } else {
        return NONE;
    }
}

std::vector<BoatTest *> YamlParser::parseYaml(const char * yaml_file_path)
{
    std::vector<BoatTest *> tests;
    std::string             yaml_contents = readFile(yaml_file_path);

    ryml::Tree    tree       = ryml::parse_in_place(ryml::to_substr(yaml_contents));
    ryml::NodeRef test_array = tree["inputs"];

    for (ryml::NodeRef const & test : test_array.children()) {
        std::string              test_type_str;
        std::string              test_name;
        testType                 test_type;
        int                      test_timeout;
        std::vector<std::string> test_data;

        test["type"] >> test_type_str;
        test_type = getTestTypeFromStr(test_type_str);
        test["name"] >> test_name;
        test["timeout_sec"] >> test_timeout;
        // TODO(unknown): add parsing for test data

        BoatTest * new_test = new BoatTest(test_name, test_type, test_timeout, test_data);
        tests.push_back(new_test);
    }
    return tests;
}
