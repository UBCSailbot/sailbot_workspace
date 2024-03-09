cmake_minimum_required(VERSION 3.8)
project(diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ryml REQUIRED)


add_subdirectory(src)
target_include_directories(diagnostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(diagnostics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS diagnostics
  DESTINATION lib/${PROJECT_NAME})

ament_package()
