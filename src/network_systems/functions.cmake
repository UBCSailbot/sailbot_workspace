# Create module library
function(make_lib module srcs link_libs inc_dirs compile_defs)
    add_library(${module} INTERFACE ${srcs})
    target_compile_definitions(${module} INTERFACE ${compile_defs})
    target_link_libraries(${module} INTERFACE ${link_libs})
    target_include_directories(
        ${module} INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${CMAKE_SOURCE_DIR}/lib
        ${inc_dirs}
    )
    add_dependencies(${module} ${AUTOGEN_TARGETS})
endfunction()

# Create module ROS executable
function(make_exe module srcs link_libs inc_dirs ${compile_defs})
    set(bin_module bin_${module})
    add_executable(${bin_module} ${srcs})
    target_compile_definitions(${bin_module} PUBLIC ${compile_defs})
    ament_target_dependencies(${bin_module} PUBLIC ${ROS_DEPS})
    target_link_libraries(${bin_module} PUBLIC ${link_libs} boost_program_options net_node)
    target_include_directories(
        ${bin_module} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${CMAKE_SOURCE_DIR}/lib
        ${inc_dirs}
        ${net_node_inc_dir}
    )
    add_dependencies(${bin_module} ${AUTOGEN_TARGETS})
    install(TARGETS ${bin_module} DESTINATION lib/${PROJECT_NAME})
    # Rename the output binary to just be the module name
    set_target_properties(${bin_module} PROPERTIES OUTPUT_NAME ${module})
endfunction()

# Create unit test
function(make_unit_test module srcs link_libs inc_dirs compile_defs)
    if(UNIT_TEST)
        set(test_module test_${module})
        add_executable(${test_module} ${srcs})
        target_compile_definitions(${test_module} PUBLIC ${compile_defs})
        ament_target_dependencies(${test_module} PUBLIC ${ROS_DEPS})
        target_include_directories(
            ${test_module} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/inc
            ${CMAKE_SOURCE_DIR}/lib
            ${inc_dirs}
        )
        target_link_libraries(${test_module} PUBLIC ${GTEST_LINK_LIBS} ${link_libs})
        add_dependencies(${test_module} ${AUTOGEN_TARGETS})
        # Make the unit test runnable with CTest (invoked via test.sh)
        add_test(NAME ${test_module} COMMAND ${test_module})
        set_tests_properties(${test_module} PROPERTIES TIMEOUT 60) # 1 minute per test timeout
    endif()
endfunction()
