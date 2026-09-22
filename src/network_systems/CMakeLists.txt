cmake_minimum_required(VERSION 3.28)

set(CLANG_VERSION "16")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-${CLANG_VERSION}" CACHE FILEPATH "Network Systems C++ compiler")

project(network_systems LANGUAGES CXX)
include(functions.cmake)

# Options and constants
option(STATIC_ANALYSIS "Enable clang-tidy checks" ON)
option(UNIT_TEST "Enable unit tests" ON)
# Configure the compiler
if(STATIC_ANALYSIS)
  set(CMAKE_CXX_CLANG_TIDY "clang-tidy-${CLANG_VERSION}")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

if(NOT CMAKE_GENERATOR MATCHES "Ninja")
  message(FATAL_ERROR "Network Systems C++ modules require the Ninja generator")
endif()

if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 16)
  message(FATAL_ERROR "Network Systems C++ modules require Clang 16 or newer")
endif()

# Configure build flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(coverage_flags
      "-fprofile-arcs -ftest-coverage -fprofile-instr-generate -fcoverage-mapping")
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS_DEBUG}"
      # ${coverage_flags} Needs to be implemented
  )
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endif()

# Enable all warnings as errors except nested-anon-types, as it allows us to
# declare an unnamed struct within an anonymous union, which is allowed in the
# C++ standard. This appears to be a warning specific to clang++ with
# -Wpedantic, and does not appear with g++ or MSVC
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -Wno-nested-anon-types -pthread"
)
message(WARNING
    "Building Network Systems with build type '${CMAKE_BUILD_TYPE}' "
    "and flags: '${CMAKE_CXX_FLAGS}'")

# ROS dependencies
set(ROS_DEPS
    rclcpp
    std_msgs
    custom_interfaces
    std_srvs
)
find_package(ament_cmake REQUIRED)
foreach(dep IN LISTS ROS_DEPS)
  find_package(${dep} REQUIRED)
endforeach()

# Boost
find_package(Boost 1.74.0 REQUIRED
    COMPONENTS
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
