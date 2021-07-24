add_library(robot_plans
        plan_base.h plan_base.cc
        joint_space_trajectory_plan.h joint_space_trajectory_plan.cc
        task_space_trajectory_plan.h task_space_trajectory_plan.cc
        admittance_trajectory_plan.h admittance_trajectory_plan.cc
        hybrid_trajectory_plan.h hybrid_trajectory_plan.cc
        rigid_trajectory_plan.h rigid_trajectory_plan.cc
        implicit_trajectory_plan.h implicit_trajectory_plan.cc)
target_link_libraries(robot_plans drake::drake Threads::Threads)
target_include_directories(robot_plans PUBLIC ${PROJECT_SOURCE_DIR}/src)

add_library(iiwa_plan_factory iiwa_plan_factory.h iiwa_plan_factory.cc
        plan_factory_base.cc plan_factory_base.h)
target_link_libraries(iiwa_plan_factory robot_plans)
target_include_directories(iiwa_plan_factory PUBLIC ${PROJECT_SOURCE_DIR}/src)
