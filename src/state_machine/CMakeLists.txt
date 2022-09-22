add_library(plan_manager_state_machine
        plan_manager_state_machine.h plan_manager_state_machine.cc
        states.h states.cc)
target_link_libraries(plan_manager_state_machine robot_plans)
target_include_directories(plan_manager_state_machine PUBLIC
        ${PROJECT_SOURCE_DIR}/src)

