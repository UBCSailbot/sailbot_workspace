cmake_minimum_required(VERSION 3.8)
project(pyompl)

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
# Enable all warnings as errors except nested-anon-types, as it allows us to declare an unnamed struct within an
# anonymous union, which is allowed in the C++ standard. This appears to be a warning specific to clang++ with
# -Wpedantic, and does not appear with g++ or MSVC
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wall -Wextra -fPIC -Wpedantic -Wno-gnu-zero-variadic-macro-arguments -pthread")
message(WARNING "Building Network Systems with build type '${CMAKE_BUILD_TYPE}' "
        "and flags: '${CMAKE_CXX_FLAGS}'")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake_python REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

include_directories("/usr/include/ompl-1.6")
include_directories("/usr/include/eigen3")
pybind11_add_module(pyompl src/ompl_bindings.cpp)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS pyompl
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_export_dependencies(pybind11)

ament_package()
