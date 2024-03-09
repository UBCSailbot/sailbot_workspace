set(module can_transceiver)

set(link_libs
)

set(inc_dirs
)

set(compile_defs
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_frame_parser.cpp
)

# Create module ROS executable
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver_ros_intf.cpp
)
make_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_can_transceiver.cpp
)
make_unit_test(${module} "${test_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
