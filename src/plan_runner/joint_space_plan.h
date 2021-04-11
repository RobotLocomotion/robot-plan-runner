#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "plan_base.h"

class JointSpacePlan : public PlanBase {
public:
  JointSpacePlan(drake::trajectories::PiecewisePolynomial<double> q_traj,
  drake::multibody::MultibodyPlant<double> *plant)
      : q_traj_(std::move(q_traj)) {plant_ = plant;};
  void Step(const State &state, double control_period, double t,
            Command *cmd) const override;

private:
  const drake::trajectories::PiecewisePolynomial<double> q_traj_;
};
