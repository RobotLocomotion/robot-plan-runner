cmake_minimum_required(VERSION 3.10.2)
project(robot-plan-runner-lcm-types)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)

find_package(lcm REQUIRED)
find_package(PythonInterp REQUIRED)
include(${LCM_USE_FILE})

set(LCM_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE PATH
        "include directory for generated c++ headers." FORCE)

lcm_wrap_types(
    CPP_HEADERS lcm_cpp_headers CPP11
    PYTHON_SOURCES lcm_python_sources
    lcmt_test.lcm)

lcm_add_library(${PROJECT_NAME} CPP ${lcm_cpp_headers})
lcm_install_headers(DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} ${lcm_cpp_headers})
lcm_install_python(DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} ${lcm_python_sources})
