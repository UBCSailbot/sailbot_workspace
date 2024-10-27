cmake_minimum_required(VERSION 3.8)
project(sailompl)

# Set Python and Pybind11 options
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Find the required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Include directories for OMPL
include_directories(/usr/include/ompl-1.6)
include_directories(/usr/include/eigen3)

# Add the source file containing the bindings
set(SOURCE_FILES src/sailompl.cpp)

# Define the Python module
pybind11_add_module(${PROJECT_NAME} MODULE ${SOURCE_FILES})


# Link OMPL libraries (assuming shared libraries are in /usr/lib)
target_link_libraries(${PROJECT_NAME} PRIVATE /usr/lib/libompl.so)

# Specify include directories for OMPL and Pybind11
target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON3_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${pybind11_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE /usr/include/ompl-1.6)

# Set the module name for Python
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

# Install the Python module in the correct location
install(TARGETS ${PROJECT_NAME} DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")



# Register as an ament package
ament_package()

# cmake_minimum_required(VERSION 3.8)
# project(sailompl)


# # Enable all warnings as errors except nested-anon-types, as it allows us to declare an unnamed struct within an
# # anonymous union, which is allowed in the C++ standard. This appears to be a warning specific to clang++ with
# # -Wpedantic, and does not appear with g++ or MSVC
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wall -Wextra -fPIC -Wpedantic -Wno-gnu-zero-variadic-macro-arguments -pthread")
# message(WARNING "Building Network Systems with build type '${CMAKE_BUILD_TYPE}' "
#         "and flags: '${CMAKE_CXX_FLAGS}'")

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# find_package(ament_cmake_python REQUIRED)
# find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter Development)
# set(PYBIND11_PYTHON_VERSION 3.10 CACHE STRING "")
# find_package(pybind11 CONFIG)
# find_package(Eigen3 REQUIRED)
# find_package(ompl REQUIRED)

# include_directories("/usr/include/ompl-1.6")
# include_directories("/usr/include/eigen3")
# pybind11_add_module(sailompl src/sailompl.cpp)

# # ament_python_install_package(${PROJECT_NAME})

# install(
#   TARGETS sailompl
#   DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
# )

# target_include_directories(sailompl PRIVATE /usr/include/ompl-1.6/ompl/base  /usr/include/ompl-1.6/ompl/base/objectives /usr/include/ompl-1.6/ompl/base/spaces /usr/include/ompl-1.6/ompl/geometric /usr/include/ompl-1.6/ompl/util)

# target_link_libraries(sailompl PRIVATE pybind11::module pybind11::lto)
# set_target_properties(sailompl PROPERTIES OUTPUT_NAME sailompl)
# set_target_properties(sailompl PROPERTIES CXX_VISIBILITY_PRESET "hidden"
#                                          CUDA_VISIBILITY_PRESET "hidden")

# ament_export_dependencies(pybind11)

# ament_package()
