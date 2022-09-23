add_library(robot_plans
        plan_base.h plan_base.cc
        joint_space_trajectory_plan.h joint_space_trajectory_plan.cc
        task_space_trajectory_plan.h task_space_trajectory_plan.cc)
target_link_libraries(robot_plans drake::drake)
target_include_directories(robot_plans PUBLIC ${PROJECT_SOURCE_DIR}/src)

add_library(plan_factory_base plan_factory_base.h plan_factory_base.cc)
target_link_libraries(plan_factory_base robot_plans)
target_include_directories(plan_factory_base PUBLIC ${PROJECT_SOURCE_DIR}/src)

add_library(bimanual_iiwa_plan_factory bimanual_iiwa_plan_factory.h
        bimanual_iiwa_plan_factory.cc)
target_link_libraries(bimanual_iiwa_plan_factory robot_plans plan_factory_base)
target_include_directories(bimanual_iiwa_plan_factory PUBLIC ${PROJECT_SOURCE_DIR}/src)

add_library(iiwa_plan_factory iiwa_plan_factory.h iiwa_plan_factory.cc)
target_link_libraries(iiwa_plan_factory robot_plans plan_factory_base)
target_include_directories(iiwa_plan_factory PUBLIC ${PROJECT_SOURCE_DIR}/src)
