add_library(robot_plans plan_base.h plan_base.cc
        joint_space_plan.h joint_space_plan.cc)
target_link_libraries(robot_plans drake::drake)

add_library(plan_manager_state_machine
        plan_manager_state_machine.h plan_manager_state_machine.cc
        state_init.h state_init.cc
        state_idle.h state_idle.cc)
target_link_libraries(plan_manager_state_machine robot_plans)

add_library(plan_manager plan_manager.h plan_manager.cc)
target_link_libraries(plan_manager plan_manager_state_machine)

add_executable(run_zmq_example run_zmq_example.cc)
target_link_libraries(run_zmq_example cppzmq)