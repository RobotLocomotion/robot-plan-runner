add_library(plan_manager_state_machine
        plan_manager_state_machine.h plan_manager_state_machine.cc
        state_init.h state_init.cc
        state_idle.h state_idle.cc
        state_running.h state_running.cc
        state_error.h state_error.cc)
target_link_libraries(plan_manager_state_machine robot_plans)
