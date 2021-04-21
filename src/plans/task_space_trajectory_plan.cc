#include "task_space_trajectory_plan.h"

using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::manipulation::planner::DoDifferentialInverseKinematics;

void TaskSpaceTrajectoryPlan::Step(const State &state, double control_period,
                                   double t, Command *cmd) const {

  // 1. Update diffik mbp with the current status of the robot.
  plant_->SetPositions(plant_context_.get(), state.q);
  // TODO(terry-suh): Get this from a config file.
  const drake::multibody::Frame<double> &frame_E =
      plant_->GetFrameByName("iiwa_link_7");

  // 2. Ask diffik to solve for desired position.
  drake::math::RigidTransformd X_WE_desired(quat_traj_.orientation(t),
                                            xyz_traj_.value(t));

  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      *plant_, *plant_context_, X_WE_desired.GetAsIsometry3(), frame_E,
      *params_);

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
