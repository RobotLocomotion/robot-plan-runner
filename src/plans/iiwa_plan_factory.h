#pragma once

#include "plan_factory_base.h"
#include <yaml-cpp/yaml.h>

class IiwaPlanFactory : public PlanFactory {
public:
  IiwaPlanFactory(YAML::Node config_);

  ~IiwaPlanFactory() override = default;

  [[nodiscard]] std::unique_ptr<PlanBase>
  MakePlan(const drake::lcmt_robot_plan &msg_plan,
           YAML::Node config) const override;

private:
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeJointSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan,
                               YAML::Node config) const;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeTaskSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan,
                              YAML::Node config) const;
};
