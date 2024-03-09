#include "diagnostics.h"

App::App()
{
    CommonUI *   new_ui          = new CommonUI();
    YamlParser * new_yaml_parser = new YamlParser();

    ui          = new_ui;
    yaml_parser = new_yaml_parser;
}

int App::appGetUserSelection(std::string * selection) { return 0; }

App::~App()
{
    delete ui;
    delete yaml_parser;
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    int status;
    App diagnostics_app;

    while (true) {
        std::string user_select;
        status = diagnostics_app.appGetUserSelection(&user_select);
        if (status) {
            std::cout << "Error in parsing user input. Exiting." << std::endl;
        }

        if (user_select == "q") {
            break;
        } else {
            std::cout << "Option " << user_select << " chosen." << std::endl;
        }
    }

    return 0;
}
