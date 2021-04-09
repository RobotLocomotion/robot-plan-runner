add_library(plan_base plan_base.h plan_base.cc)
target_link_libraries(plan_base drake::drake)

add_library(plan_manager_state plan_manager_state.h plan_manager_state.cc)
target_link_libraries(plan_manager_state plan_base)

add_library(plan_manager plan_manager.h plan_manager.cc)
target_link_libraries(plan_manager plan_manager_state)

add_executable(run_zmq_example run_zmq_example.cc)
target_link_libraries(run_zmq_example cppzmq)