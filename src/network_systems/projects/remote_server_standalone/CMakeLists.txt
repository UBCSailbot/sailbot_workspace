cmake_minimum_required(VERSION 3.10)
project(remote_server_standalone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MONGOCXX REQUIRED libmongocxx)
pkg_check_modules(BSONCXX REQUIRED libbsoncxx)
find_package(CURL REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${MONGOCXX_INCLUDE_DIRS}
    ${BSONCXX_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    inc
)

link_directories(
    ${MONGOCXX_LIBRARY_DIRS}
    ${BSONCXX_LIBRARY_DIRS}
)


# Explicitly list protobuf-generated source files to ensure they are linked
set(PROTO_SRCS
    src/global_path.pb.cc
    src/sensors.pb.cc
    src/waypoint.pb.cc
)

file(GLOB APP_SRCS src/*.cpp)

# Remove the protobuf-generated .cc files from APP_SRCS to avoid double inclusion
list(REMOVE_ITEM APP_SRCS ${PROTO_SRCS})

set(SOURCES ${APP_SRCS} ${PROTO_SRCS})

add_executable(remote_server ${SOURCES})

# Link libraries
 target_link_libraries(remote_server
    ${Boost_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${MONGOCXX_LIBRARIES}
    ${BSONCXX_LIBRARIES}
    ${CURL_LIBRARIES}
)
