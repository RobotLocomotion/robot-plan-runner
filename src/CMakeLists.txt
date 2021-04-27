add_subdirectory(plans)
add_subdirectory(state_machine)
add_subdirectory(plan_manager_system)


add_library(iiwa_plan_manager iiwa_plan_manager.h iiwa_plan_manager.cc)
target_link_libraries(iiwa_plan_manager
        plan_manager_state_machine cppzmq iiwa_plan_factory Threads::Threads)

add_executable(run_zmq_example run_zmq_example.cc)
target_link_libraries(run_zmq_example cppzmq iiwa_plan_factory yaml-cpp)

add_executable(sandbox sandbox.cc)
target_link_libraries(sandbox  drake::drake Threads::Threads)
target_include_directories(sandbox PUBLIC ${LCM_INCLUDE_PATH})


add_executable(run_plan_manager run_plan_manager.cc)
target_link_libraries(run_plan_manager iiwa_plan_manager yaml-cpp)

add_executable(run_plan_manager_system run_plan_manager_system.cc)
target_link_libraries(run_plan_manager_system
        iiwa_plan_manager_hardware_interface yaml-cpp)

add_test(NAME import_all_test COMMAND
  "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/import_all_test.py"
)
set_tests_properties(import_all_test PROPERTIES
  ENVIRONMENT "PYTHONPATH=${PYTHONPATH}"
)
