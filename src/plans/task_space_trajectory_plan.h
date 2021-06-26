#pragma once

#include <cmath>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/context.h"

#include "plans/plan_base.h"

class TaskSpaceTrajectoryPlan : public PlanBase {
public:
  TaskSpaceTrajectoryPlan(
      drake::trajectories::PiecewiseQuaternionSlerp<double> quat_traj,
      drake::trajectories::PiecewisePolynomial<double> xyz_traj,
      drake::math::RigidTransformd X_ET,
      const drake::multibody::MultibodyPlant<double> *plant,
      const drake::multibody::Frame<double> &frame_E,
      double control_time_step)
      : PlanBase(plant), quat_traj_(std::move(quat_traj)),
        X_ET_(std::move(X_ET)), xyz_traj_(std::move(xyz_traj)),
        frame_E_(frame_E) {

    params_ = std::make_unique<
        drake::manipulation::planner::DifferentialInverseKinematicsParameters>(
        plant_->num_positions(), plant_->num_velocities());
    params_->set_timestep(control_time_step);

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
  const drake::math::RigidTransformd X_ET_;

  // frame of end-effector body + offset.
  const drake::multibody::Frame<double> &frame_E_;

  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  std::unique_ptr<
      drake::manipulation::planner::DifferentialInverseKinematicsParameters>
      params_;
};
