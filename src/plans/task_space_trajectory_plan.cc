#include "task_space_trajectory_plan.h"

using drake::MatrixX;
using drake::Vector3;
using drake::Vector6;
using drake::manipulation::planner::ComputePoseDiffInCommonFrame;
using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::manipulation::planner::internal::DoDifferentialInverseKinematics;

TaskSpaceTrajectoryPlan::TaskSpaceTrajectoryPlan(
    drake::trajectories::PiecewiseQuaternionSlerp<double> quat_traj,
    drake::trajectories::PiecewisePolynomial<double> xyz_traj,
    drake::math::RigidTransformd X_ET,
    const drake::multibody::MultibodyPlant<double> *plant,
    const drake::multibody::Frame<double> &frame_E, double control_time_step,
    Eigen::VectorXd nominal_joint_position)
    : PlanBase(plant), quat_traj_(std::move(quat_traj)), X_ET_(std::move(X_ET)),
      xyz_traj_(std::move(xyz_traj)), frame_E_(frame_E) {

  params_ = std::make_unique<
      drake::manipulation::planner::DifferentialInverseKinematicsParameters>(
      plant_->num_positions(), plant_->num_velocities());
  params_->set_timestep(control_time_step);
  params_->set_nominal_joint_position(nominal_joint_position);
  plant_context_ = plant_->CreateDefaultContext();
}

void TaskSpaceTrajectoryPlan::Step(const State &state, double control_period,
                                   double t, Command *cmd) const {

  // 1. Update diffik mbp with the current status of the robot.
  plant_->SetPositions(plant_context_.get(), state.q);

  // 2. Ask diffik to solve for desired position.
  const drake::math::RigidTransformd X_WT_desired(quat_traj_.orientation(t),
                                                  xyz_traj_.value(t));
  const auto &frame_W = plant_->world_frame();
  const auto X_WE =
      plant_->CalcRelativeTransform(*plant_context_, frame_W, frame_E_);
  const auto X_WT = X_WE * X_ET_;

  const Vector6<double> V_WT_desired =
      ComputePoseDiffInCommonFrame(X_WT.GetAsIsometry3(),
                                   X_WT_desired.GetAsIsometry3()) /
      params_->get_timestep();

  Eigen::MatrixXd J_WT(6, plant_->num_velocities());
  plant_->CalcJacobianSpatialVelocity(
      *plant_context_, drake::multibody::JacobianWrtVariable::kV, frame_E_,
      X_ET_.translation(), frame_W, frame_W, &J_WT);

  const auto result = DoDifferentialInverseKinematics(
      state.q, state.v, X_WT, J_WT,
      drake::multibody::SpatialVelocity<double>(V_WT_desired), *params_);

  // 3. Check for errors and integrate.
  if (result.status != DifferentialInverseKinematicsStatus::kSolutionFound) {
    throw DiffIkException();
  } else {
    cmd->q_cmd = state.q + control_period * result.joint_velocities.value();
    cmd->tau_cmd = Eigen::VectorXd::Zero(7);
  }
}
