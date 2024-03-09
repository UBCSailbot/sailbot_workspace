add_executable(diagnostics diagnostics.cpp)

target_link_libraries(diagnostics PRIVATE
    UI_Common
    parseYaml
)

target_link_libraries(diagnostics PRIVATE boatTest_Common)
target_include_directories(diagnostics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(diagnostics PUBLIC YAML_TEST_PATH="${YAML_TEST_PATH}")
