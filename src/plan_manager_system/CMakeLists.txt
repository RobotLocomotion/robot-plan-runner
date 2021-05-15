add_library(iiwa_plan_manager_system
        iiwa_plan_manager_system.h iiwa_plan_manager_system.cc)
target_link_libraries(iiwa_plan_manager_system
        drake::drake)
target_include_directories(iiwa_plan_manager_system
        PUBLIC ${PROJECT_SOURCE_DIR}/src)

add_library(iiwa_plan_manager_hardware_interface
        iiwa_plan_manager_hardware_interface.h
        iiwa_plan_manager_hardware_interface.cc)
target_link_libraries(iiwa_plan_manager_hardware_interface
        iiwa_plan_manager_system)
