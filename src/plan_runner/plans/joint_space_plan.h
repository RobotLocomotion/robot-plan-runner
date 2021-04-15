#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "plan_base.h"

class JointSpacePlan : public PlanBase {
public:
  JointSpacePlan(drake::trajectories::PiecewisePolynomial<double> q_traj,
  const drake::multibody::MultibodyPlant<double> *plant)
      : PlanBase(plant), q_traj_(std::move(q_traj)) {};
  ~JointSpacePlan() override = default;
  void Step(const State &state, double control_period, double t,
            Command *cmd) const override;
  [[nodiscard]] double duration() const override {
    return q_traj_.end_time() - q_traj_.start_time();};
private:
  const drake::trajectories::PiecewisePolynomial<double> q_traj_;
};
