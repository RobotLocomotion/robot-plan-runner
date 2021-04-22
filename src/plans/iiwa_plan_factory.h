#pragma once

#include "plan_factory_base.h"
#include <yaml-cpp/yaml.h>

class IiwaPlanFactory : public PlanFactory {
public:
  IiwaPlanFactory(const YAML::Node &config);

  ~IiwaPlanFactory() override = default;

  [[nodiscard]] std::unique_ptr<PlanBase>
  MakePlan(const drake::lcmt_robot_plan &msg_plan) const override;

private:
  const YAML::Node &config_;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeJointSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeTaskSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;
};
