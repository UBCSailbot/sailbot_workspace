set(module local_transceiver)

set(link_libs
    ${PROTOBUF_LINK_LIBS}
)

set(inc_dirs
    ${PROTOBUF_INCLUDE_PATH}
)

set(compile_defs
    LOCAL_TRANSCEIVER_TEST_PORT="$ENV{LOCAL_TRANSCEIVER_TEST_PORT}"
    RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH="$ENV{ROS_WORKSPACE}/scripts/run_virtual_iridium.sh"
    RUN_HTTP_ECHO_SERVER_CMD="python3 $ENV{ROS_WORKSPACE}/src/network_systems/scripts/http_echo_server.py"
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/local_transceiver.cpp
)

# Create module ROS executable
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/local_transceiver_ros_intf.cpp
)
make_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

# Create unit test
set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_local_transceiver.cpp
)
make_unit_test(${module} "${test_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
