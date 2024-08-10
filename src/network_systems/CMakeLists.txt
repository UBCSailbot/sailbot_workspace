cmake_minimum_required(VERSION 3.10)
project(network_systems)
include(functions.cmake)

# Options and constants
option(STATIC_ANALYSIS "Enable clang-tidy checks" ON)
option(UNIT_TEST "Enable unit tests" ON)
set(CLANG_VERSION "14")

# Configure the compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-${CLANG_VERSION}")
if(STATIC_ANALYSIS)
    set(CMAKE_CXX_CLANG_TIDY "clang-tidy-${CLANG_VERSION}")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Configure build flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(coverage_flags "-fprofile-arcs -ftest-coverage -fprofile-instr-generate -fcoverage-mapping")
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS_DEBUG}"
        # ${coverage_flags} Needs to be implemented
    )
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endif()
# Enable all warnings as errors except unused parameter because auto-generated protobuf files trigger it,
# and clang-tidy can cover it without false-flagging the protobuf files.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter -pthread")
message(WARNING "Building Network Systems with build type '${CMAKE_BUILD_TYPE}' "
        "and flags: '${CMAKE_CXX_FLAGS}'")

# ROS dependencies
set(ROS_DEPS
    rclcpp
    std_msgs
    custom_interfaces
)
find_package(ament_cmake REQUIRED)
foreach(dep IN LISTS ROS_DEPS)
    find_package(${dep} REQUIRED)
endforeach()


# Boost
find_package(Boost 1.74.0 COMPONENTS REQUIRED
    program_options
    serialization
    system
    thread
)

# MongoDB
find_package(mongocxx REQUIRED)
find_package(bsoncxx REQUIRED)

# Protobuf
find_package(Protobuf REQUIRED)
set(PROTOBUF_LINK_LIBS
    ${Protobuf_LIBRARIES}
    protofiles
)

# Googletest (installed by sailbot_workspace Docker config)
if(UNIT_TEST)
    find_package(GTest CONFIG REQUIRED)
    set(GTEST_LINK_LIBS gtest gtest_main)
endif()

# CURL
find_package(CURL REQUIRED)

# Add src directories
add_subdirectory(lib)
add_subdirectory(projects)

# Install launch files
install(DIRECTORY launch DESTINATION share/network_systems)

ament_package()
