add_library(parseYaml parse_yaml.cpp parse_yaml.h)
target_include_directories(parseYaml PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(parseYaml PRIVATE boatTest_Common)
target_link_libraries(parseYaml PRIVATE ryml::ryml)

target_compile_definitions(
    parseYaml PUBLIC YAML_TEST_PATH_1="${YAML_TEST_PATH_1}"
)
