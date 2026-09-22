cmake_minimum_required(VERSION 3.28)

set(CMAKE_CXX_COMPILER "/usr/bin/clang++-16" CACHE FILEPATH "Remote server C++ compiler")
project(remote_server_standalone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

if(NOT CMAKE_GENERATOR MATCHES "Ninja")
    message(FATAL_ERROR "Remote server C++ modules require the Ninja generator")
endif()

if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 16)
    message(FATAL_ERROR "Remote server C++ modules require Clang 16 or newer")
endif()

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MONGOCXX REQUIRED libmongocxx)
pkg_check_modules(BSONCXX REQUIRED libbsoncxx)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${MONGOCXX_INCLUDE_DIRS}
    ${BSONCXX_INCLUDE_DIRS}
    inc
)
link_directories(
    ${MONGOCXX_LIBRARY_DIRS}
    ${BSONCXX_LIBRARY_DIRS}
)

set(PROTO_SRCS
    src/global_path.pb.cc
    src/sensors.pb.cc
    src/waypoint.pb.cc
)

# App logic shared by the server and the tests (exclude main.cpp and the test-only helper)
set(APP_LOGIC_SRCS
    src/remote_transceiver.cpp
    src/sailbot_db.cpp
)
set(APP_MODULE_SRCS
    src/remote_transceiver.cppm
    src/sailbot_db.cppm
    src/shared_constants.cppm
)

add_library(remote_server_lib STATIC ${APP_LOGIC_SRCS} ${PROTO_SRCS})
target_sources(
    remote_server_lib PUBLIC
    FILE_SET cxx_modules TYPE CXX_MODULES
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
    FILES ${APP_MODULE_SRCS}
)
target_compile_features(remote_server_lib PUBLIC cxx_std_20)
target_link_libraries(remote_server_lib
    ${Boost_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${MONGOCXX_LIBRARIES}
    ${BSONCXX_LIBRARIES}
)

add_executable(remote_server src/main.cpp)
target_compile_features(remote_server PRIVATE cxx_std_20)
target_link_libraries(remote_server remote_server_lib)

# ---- Tests ----
option(BUILD_TESTING "Build the standalone test suite" ON)
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    enable_testing()
    add_executable(remote_server_tests
        test/test_remote_transceiver.cpp
        test/test_db_helper.cpp
    )
    target_sources(
        remote_server_tests PRIVATE
        FILE_SET cxx_modules TYPE CXX_MODULES
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
            test/test_db_helper.cppm
            test/utils.cppm
    )
    target_compile_features(remote_server_tests PRIVATE cxx_std_20)
    # test/ holds the test-only module interfaces.
    target_include_directories(remote_server_tests PRIVATE test)
    target_link_libraries(remote_server_tests
        remote_server_lib
        GTest::gtest
        GTest::gtest_main
        ${Boost_LIBRARIES}
        ${Protobuf_LIBRARIES}
        ${MONGOCXX_LIBRARIES}
        ${BSONCXX_LIBRARIES}
    )
    add_test(NAME remote_server_tests COMMAND remote_server_tests)
endif()
