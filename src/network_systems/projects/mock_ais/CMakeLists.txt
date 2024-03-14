set(module mock_ais)

# define external dependencies with link_libs and inc_dirs variables
set(link_libs
)

set(inc_dirs
)

set(compile_defs
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/mock_ais.cpp
)

# Create module ROS executable
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/mock_ais_ros_intf.cpp
)
make_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

# Create unit test
set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_mock_ais.cpp
)
make_unit_test(${module} "${test_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
