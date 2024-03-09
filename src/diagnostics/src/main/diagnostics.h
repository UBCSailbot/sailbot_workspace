#ifndef DIAGNOSTICS_H_
#define DIAGNOSTICS_H_

/* Include Files */
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "boatTest_common.h"
#include "commonUI.h"
#include "parse_yaml.h"

/* Classes */
class App
{
public:
    CommonUI *   ui;
    YamlParser * yaml_parser;

    App();
    int appGetUserSelection(std::string * selection);
    ~App();
};

#endif
