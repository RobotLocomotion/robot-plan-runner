add_library(iiwa_plan_manager_system
        iiwa_plan_manager_system.h iiwa_plan_manager_system.cc)
target_link_libraries(iiwa_plan_manager_system
        plan_manager_state_machine iiwa_plan_factory)

add_library(iiwa_plan_manager_hardware_interface
        iiwa_plan_manager_hardware_interface.h
        iiwa_plan_manager_hardware_interface.cc)
target_link_libraries(iiwa_plan_manager_hardware_interface
        iiwa_plan_manager_system)
