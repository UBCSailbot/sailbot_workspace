#ifndef PARSE_YAML_H_
#define PARSE_YAML_H_

/* Include Files */
#include <fstream>
#include <ios>
#include <iostream>
#include <ryml.hpp>
#include <ryml_std.hpp>
#include <sstream>

#include "boatTest_common.h"

/* Yaml File Parser*/
class YamlParser
{
public:
    std::vector<BoatTest *> parseYaml(const char * yaml_file_path);
};

#endif
