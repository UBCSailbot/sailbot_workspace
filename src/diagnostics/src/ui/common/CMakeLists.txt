add_library(UI_Common commonUI.cpp commonUI.h)
target_link_libraries(UI_Common PUBLIC parseYaml boatTest_Common)
target_include_directories(UI_Common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
