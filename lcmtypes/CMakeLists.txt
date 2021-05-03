cmake_minimum_required(VERSION 3.10.2)
project(robot-plan-runner-lcmtypes)

find_package(lcm REQUIRED)
find_package(PythonInterp REQUIRED)
include(${LCM_USE_FILE})

lcm_wrap_types(
    CPP_HEADERS lcm_cpp_headers CPP11
    PYTHON_SOURCES lcm_python_sources
    lcmt_plan_status.lcm lcmt_plan_status_constants.lcm)

lcm_add_library(${PROJECT_NAME} CPP ${lcm_cpp_headers})
add_custom_target(GenLcmtypes ALL DEPENDS ${PROJECT_NAME}.sources)
