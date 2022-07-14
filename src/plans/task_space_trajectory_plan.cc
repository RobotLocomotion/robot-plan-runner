#include "task_space_trajectory_plan.h"

#include <iostream>

using drake::MatrixX;
using drake::Vector3;
using drake::Vector6;
using drake::manipulation::planner::ComputePoseDiffInCommonFrame;
using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::manipulation::planner::internal::DoDifferentialInverseKinematics;

using std::cout;
using std::endl;

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
      ComputePoseDiffInCommonFrame(X_WT, X_WT_desired) /
      params_->get_timestep();

  MatrixX<double> J_WT(6, plant_->num_velocities());
  plant_->CalcJacobianSpatialVelocity(
      *plant_context_, drake::multibody::JacobianWrtVariable::kV, frame_E_,
      X_ET_.translation(), frame_W, frame_W, &J_WT);

  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      state.q, state.v, X_WT, J_WT,
      drake::multibody::SpatialVelocity<double>(V_WT_desired), *params_);

  // 3. Check for errors and integrate.
  if (result.status != DifferentialInverseKinematicsStatus::kSolutionFound) {
    // Set the command to NAN so that state machine will detect downstream and
    // go to error state.
    cmd->q_cmd = NAN * Eigen::VectorXd::Zero(7);
    // TODO(terry-suh): how do I tell the use that the state machine went to
    // error because of this precise reason? Printing the error message here
    // seems like a good start, but we'll need to handle this better.
    std::cout << "DoDifferentialKinematics Failed to find a solution."
              << std::endl;
  } else {
    cmd->q_cmd = state.q + control_period * result.joint_velocities.value();
    cmd->tau_cmd = Eigen::VectorXd::Zero(7);
  }
}
