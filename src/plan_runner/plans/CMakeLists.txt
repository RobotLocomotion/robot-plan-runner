add_library(robot_plans
        plan_base.h plan_base.cc
        joint_space_plan.h joint_space_plan.cc)
target_link_libraries(robot_plans drake::drake)