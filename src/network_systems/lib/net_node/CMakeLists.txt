set(module net_node)

set(link_libs
)

set(inc_dirs
)

set(compile_defs
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/net_node.cpp
)
set(module_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/net_node.cppm
)
make_lib(${module} "${srcs}" "${module_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
