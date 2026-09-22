set(module can_transceiver_lib)

set(link_libs
    network_common
)

set(inc_dirs
)

set(compile_defs
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_frame_parser.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_log_replayer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/mock_can_bus.cpp
)
set(module_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/can_frame_parser.cppm
    ${CMAKE_CURRENT_LIST_DIR}/src/can_log_replayer.cppm
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver.cppm
    ${CMAKE_CURRENT_LIST_DIR}/src/mock_can_bus.cppm
)
make_lib(${module} "${srcs}" "${module_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

# Create module ROS executable
set(bin_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver_ros_intf.cpp
)
set(bin_link_libs can_transceiver_lib)
make_exe(can_transceiver "${bin_srcs}" "${bin_link_libs}" "${inc_dirs}" "${compile_defs}")

set(test_srcs
    ${CMAKE_CURRENT_LIST_DIR}/test/test_can_transceiver.cpp
)
set(test_link_libs can_transceiver_lib)
make_unit_test(can_transceiver "${test_srcs}" "${test_link_libs}" "${inc_dirs}" "${compile_defs}")
