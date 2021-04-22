lcm_wrap_types(
    CPP_HEADERS cpp_headers
    PYTHON_SOURCES python_sources
    lcmt_test.lcm
)

set(RprLcmLib robot-plan-runner-lcmtypes)

lcm_add_library(${RprLcmLib} CPP ${cpp_headers})
lcm_target_link_libraries(${RprLcmLib} INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
lcm_install_headers(DESTINATION ${CMAKE_INSTALL_PREFIX} ${cpp_headers})
lcm_install_python(${python_sources})

add_custom_target(GenHeaders ALL
                  DEPENDS ${RprLcmLib}.sources)