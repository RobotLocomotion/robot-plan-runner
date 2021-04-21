add_library(robot_plans
        plan_base.h plan_base.cc
        joint_space_trajectory_plan.h joint_space_trajectory_plan.cc
        task_space_trajectory_plan.h task_space_trajectory_plan.cc)
target_link_libraries(robot_plans drake::drake)

add_library(iiwa_plan_factory iiwa_plan_factory.h iiwa_plan_factory.cc
        plan_factory_base.cc plan_factory_base.h)
target_link_libraries(iiwa_plan_factory robot_plans)
