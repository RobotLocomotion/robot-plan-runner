cmake_minimum_required(VERSION 3.10.2)
project(robot-plan-runner-lcmtypes)

find_package(lcm REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(Java)
include(${LCM_USE_FILE})

if(JAVA_FOUND)
  include(UseJava)
  set(java_args JAVA_SOURCES java_sources)
endif()


lcm_wrap_types(
    CPP_HEADERS lcm_cpp_headers CPP11
    PYTHON_SOURCES lcm_python_sources
    ${java_args}
    lcmt_test.lcm)

lcm_add_library(${PROJECT_NAME} CPP ${lcm_cpp_headers})
lcm_install_headers(DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} ${lcm_cpp_headers})
lcm_install_python(DESTINATION ${CMAKE_CURRENT_SOURCE_DIR} ${lcm_python_sources})
if(JAVA_FOUND)
  add_jar(${PROJECT_NAME}-jar
    OUTPUT_NAME ${PROJECT_NAME}
    INCLUDE_JARS lcm-java
    SOURCES ${java_sources}
  )
  # install_jar(${PROJECT_NAME} share/java)
endif()

add_custom_target(GenLcmtypes ALL DEPENDS ${PROJECT_NAME}.sources)
