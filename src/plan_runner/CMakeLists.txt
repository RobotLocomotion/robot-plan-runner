add_subdirectory(plans)

add_library(plan_manager_state_machine
        plan_manager_state_machine.h plan_manager_state_machine.cc
        state_init.h state_init.cc
        state_idle.h state_idle.cc
        state_running.h state_running.cc state_error.cc state_error.h run_plan_manager.cc)
target_link_libraries(plan_manager_state_machine robot_plans)

add_library(iiwa_plan_manager iiwa_plan_manager.h iiwa_plan_manager.cc)
target_link_libraries(iiwa_plan_manager plan_manager_state_machine cppzmq)

add_executable(run_zmq_example run_zmq_example.cc)
target_link_libraries(run_zmq_example cppzmq iiwa_plan_factory)

find_package(Threads REQUIRED)
add_executable(sandbox sandbox.cc)
target_link_libraries(sandbox drake::drake ${CMAKE_THREAD_LIBS_INIT})

add_executable(run_plan_manager run_plan_manager.cc)
target_link_libraries(run_plan_manager iiwa_plan_manager)

