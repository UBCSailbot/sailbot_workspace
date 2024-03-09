set(module sailbot_db)

set(link_libs
    ${PROTOBUF_LINK_LIBS}
    mongo::mongocxx_shared
    mongo::bsoncxx_shared
)

set(inc_dirs
    ${PROTOBUF_INCLUDE_PATH}
    ${LIBMONGOCXX_INCLUDE_DIRS}
    ${LIBBSONCXX_INCLUDE_DIRS}
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/sailbot_db.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/util_db.cpp
)

# make sailbot_db library
make_lib(${module} "${srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
)

# Make executable
make_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

# Create unit test
set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_sailbot_db.cpp
)
make_unit_test(${module} "${test_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
