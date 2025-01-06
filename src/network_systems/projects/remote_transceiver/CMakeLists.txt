set(module remote_transceiver)

set(link_libs
    ${PROTOBUF_LINK_LIBS}
    ${CURL_LIBRARIES}
    mongo::mongocxx_shared
    mongo::bsoncxx_shared
    sailbot_db
)

set(inc_dirs
    ${PROTOBUF_INCLUDE_PATH}
    ${LIBMONGOCXX_INCLUDE_DIRS}
    ${LIBBSONCXX_INCLUDE_DIRS}
    ${SAILBOT_DB_INC_DIR}
    ${CURL_INCLUDE_DIR}
)

set(compile_defs
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/remote_transceiver.cpp
)

set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/remote_transceiver_ros_intf.cpp
)

# Make executable
make_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

# Create unit test
set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_remote_transceiver.cpp
)
make_unit_test(${module} "${test_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
