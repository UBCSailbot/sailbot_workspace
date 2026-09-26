set(module local_transceiver_lib)

set(link_libs
    ${PROTOBUF_LINK_LIBS}
    CURL::libcurl
    network_common
)

set(inc_dirs
    ${PROTOBUF_INCLUDE_PATH}
    ${CURL_INCLUDE_DIR}
)

set(compile_defs
    BOOST_ASIO_USE_TS_EXECUTOR_AS_DEFAULT
    LOCAL_TRANSCEIVER_TEST_PORT="$ENV{LOCAL_TRANSCEIVER_TEST_PORT}"
    RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH="$ENV{ROS_WORKSPACE}/scripts/run_virtual_iridium.sh"
    RUN_HTTP_ECHO_SERVER_CMD="python3 $ENV{ROS_WORKSPACE}/src/network_systems/scripts/http_echo_server.py"
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/local_transceiver.cpp
)
set(module_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/at_cmds.cppm
    ${CMAKE_CURRENT_LIST_DIR}/src/local_transceiver.cppm
)
make_lib(${module} "${srcs}" "${module_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

# Create module ROS executable
set(bin_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/local_transceiver_ros_intf.cpp
)
set(bin_link_libs local_transceiver_lib)
make_exe(local_transceiver "${bin_srcs}" "${bin_link_libs}" "${inc_dirs}" "${compile_defs}")

# Create unit test
set(test_srcs
    ${CMAKE_CURRENT_LIST_DIR}/test/test_local_transceiver.cpp
)
set(test_link_libs local_transceiver_lib)
make_unit_test(local_transceiver "${test_srcs}" "${test_link_libs}" "${inc_dirs}" "${compile_defs}")
