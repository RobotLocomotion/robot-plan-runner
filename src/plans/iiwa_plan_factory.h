#pragma once
#include <yaml-cpp/yaml.h>

#include "plans/plan_factory_base.h"

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
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeAdmittanceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;  
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeHybridTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;    
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeImplicitTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;      
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeRigidTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;        
};
