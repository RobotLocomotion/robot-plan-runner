#pragma once
#include <yaml-cpp/yaml.h>

#include "plans/plan_factory_base.h"

class BimanualIiwaPlanFactory : public PlanFactory {
public:
  BimanualIiwaPlanFactory(const YAML::Node &config);

  ~BimanualIiwaPlanFactory() override = default;

  [[nodiscard]] std::unique_ptr<PlanBase>
  MakePlan(const drake::lcmt_robot_plan &msg_plan) const override;

private:
  const YAML::Node &config_;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeJointSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;
  // [[nodiscard]] std::unique_ptr<PlanBase>
  // MakeTaskSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;
};
