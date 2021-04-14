add_library(robot_plans
        plan_base.h plan_base.cc
        joint_space_plan.h joint_space_plan.cc)
target_link_libraries(robot_plans drake::drake)

add_library(iiwa_plan_factory iiwa_plan_factory.h iiwa_plan_factory.cc)
target_link_libraries(iiwa_plan_factory robot_plans)
