#pragma once

#include <cmath>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/context.h"

#include "plan_base.h"

class TaskSpaceTrajectoryPlan : public PlanBase {
public:
  TaskSpaceTrajectoryPlan(
      drake::trajectories::PiecewiseQuaternionSlerp<double> quat_traj,
      drake::trajectories::PiecewisePolynomial<double> xyz_traj,
      const drake::multibody::MultibodyPlant<double> *plant, YAML::Node config)
      : PlanBase(plant, config), quat_traj_(std::move(quat_traj)),
        xyz_traj_(std::move(xyz_traj)) {

    params_ = std::make_unique<
        drake::manipulation::planner::DifferentialInverseKinematicsParameters>(
        plant_->num_positions(), plant_->num_velocities());

    plant_context_ = plant_->CreateDefaultContext();
  }

  ~TaskSpaceTrajectoryPlan() override = default;

  void Step(const State &state, double control_period, double t,
            Command *cmd) const override;

  [[nodiscard]] double duration() const override {
    return xyz_traj_.end_time() - xyz_traj_.start_time();
  };

private:
  const drake::trajectories::PiecewiseQuaternionSlerp<double> quat_traj_;
  const drake::trajectories::PiecewisePolynomial<double> xyz_traj_;

  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  std::unique_ptr<
      drake::manipulation::planner::DifferentialInverseKinematicsParameters>
      params_;
};
