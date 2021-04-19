#pragma once

#include "plan_factory_base.h"


class IiwaPlanFactory : public PlanFactory {
public:
  IiwaPlanFactory();

  ~IiwaPlanFactory() override = default;

  [[nodiscard]] std::unique_ptr<PlanBase>
  MakePlan(const drake::lcmt_robot_plan &msg_plan) const override;

private:
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeJointSpacePlan(const drake::lcmt_robot_plan &msg_plan) const;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeTaskSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;
};
