#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake_lcmtypes/drake/lcmt_robot_plan.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"

#include "plans/plan_base.h"

class PlanFactory {
public:
  virtual ~PlanFactory() = default;

  [[nodiscard]] virtual std::unique_ptr<PlanBase>

  MakePlan(const drake::lcmt_robot_plan &msg_plan) const = 0;

  [[nodiscard]] const drake::multibody::MultibodyPlant<double> &
  get_plant() const {
    return *plant_;
  };

protected:
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_{nullptr};
};
