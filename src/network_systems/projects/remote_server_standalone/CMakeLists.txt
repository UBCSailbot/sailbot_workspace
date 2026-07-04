cmake_minimum_required(VERSION 3.10)
project(remote_server_standalone)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

add_library(remote_server_lib STATIC ${APP_LOGIC_SRCS} ${PROTO_SRCS})
target_link_libraries(remote_server_lib
    ${Boost_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${MONGOCXX_LIBRARIES}
    ${BSONCXX_LIBRARIES}
)

add_executable(remote_server src/main.cpp)
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
    # test/ holds the test-only helper headers (test_db_helper.h, utils.h)
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
