#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake_lcmtypes/drake/lcmt_robot_plan.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"

#include "plan_base.h"


// TODO: to support other robots, make PlanFactory an abstract class and
//  inherit IiwaPlanFactory from it.
class IiwaPlanFactory {
public:
  std::unique_ptr<PlanBase>
  MakePlan(const drake::lcmt_robot_plan &msg_plan) const;

private:
  static std::unique_ptr<PlanBase>
  MakeJointSpacePlan(const drake::lcmt_robot_plan &msg_plan);
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
};
