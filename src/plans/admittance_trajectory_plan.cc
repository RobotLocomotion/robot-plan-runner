#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"

#include "admittance_trajectory_plan.h"

using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::manipulation::planner::internal::DoDifferentialInverseKinematics;
using drake::manipulation::planner::ComputePoseDiffInCommonFrame;
using drake::Vector3;
using drake::Vector6;
using drake::MatrixX;

using std::cout;
using std::endl;

using std::sin;
using std::cos;

void AdmittanceTrajectoryPlan::Step(const State &state, double control_period,
                                   double t, Command *cmd) const {

  // 1. Update diffik mbp with the current status of the robot.
  plant_->SetPositions(plant_context_.get(), state.q);

  // 2. Ask diffik to solve for desired position.
  const drake::math::RigidTransformd X_WT_desired(quat_traj_.orientation(t),
                                            xyz_traj_.value(t));
  const auto& frame_W = plant_->world_frame();
  const auto X_WE = plant_->CalcRelativeTransform(
      *plant_context_, frame_W, frame_E_);
  const auto X_WT = X_WE * X_ET_;

  // 3. Modify X_WT_desired with a bushing relationship.


  // xyz_TC_desired is computed by Kxyz^{-1}(F_TC - Dxyz * V_TC)
  Eigen::Vector3d xyz_TC_desired;
  for (int i = 0; i < 3; i ++) {
    xyz_TC_desired[i] = -(1./ Kxyz_[i]) * (F_TC_[i+3] - Dxyz_[i] * V_TC_[i+3]);
  }

  // yaw. using w to avoid repetition with y coordinate.
  double w_TC_desired = (1./ Krpy_[2]) * (F_TC_[2] - Drpy_[2] * V_TC_[2]);
  double p_TC_desired = (1./ Krpy_[1]) * (
    F_TC_[1] * cos(w_TC_desired) - F_TC_[0] * sin(w_TC_desired) - Drpy_[1] * V_TC_[1]);
  double r_TC_desired = (1./ Krpy_[0]) * (cos(p_TC_desired) * (
    F_TC_[0] * cos(w_TC_desired) + F_TC_[1] * sin(w_TC_desired)) - 
    F_TC_[2] * sin(p_TC_desired) - Drpy_[0] * V_TC_[0]);

  // Compute rigid transforms based on both components.
  // drake::math::RollPitchYaw<double> rpy_TC_desired(
  //  r_TC_desired, p_TC_desired, w_TC_desired);
  drake::math::RollPitchYaw<double> rpy_TC_desired(0, 0, 0);
  drake::math::RigidTransformd X_TC_desired(
    drake::math::RotationMatrixd(rpy_TC_desired), xyz_TC_desired);

  auto X_WT_corrected = X_WT_desired * X_TC_ * X_TC_desired.inverse();

  const Vector6<double> V_WT_desired =
      0.1 * ComputePoseDiffInCommonFrame(
          X_WT.GetAsIsometry3(), X_WT_corrected.GetAsIsometry3()) /
          params_->get_timestep();

  MatrixX<double> J_WT(6, plant_->num_velocities());
  plant_->CalcJacobianSpatialVelocity(*plant_context_,
                                    drake::multibody::JacobianWrtVariable::kV,
                                    frame_E_, X_ET_.translation(),
                                    frame_W, frame_W, &J_WT);


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

AdmittanceTrajectoryPlan::~AdmittanceTrajectoryPlan() {
  is_running_ = false;
  // Wait for all threads to terminate.
  for (auto &a : threads_) {
    if (a.second.joinable()) {
      a.second.join();
    }
  }
}

void AdmittanceTrajectoryPlan::SubscribeForceTorque() {
  auto sub = lcm_->subscribe("FT",
    &AdmittanceTrajectoryPlan::HandleForceTorqueStatus, this);
  sub->setQueueCapacity(1);
  while(true) {
    if (lcm_->handleTimeout(10) < 0) {
      break;
    }
    if (!is_running_) {
      break;
    }
  }
}

void AdmittanceTrajectoryPlan::SubscribeVelocity() {
  auto sub = lcm_->subscribe("RELATIVE_VELOCITY",
    &AdmittanceTrajectoryPlan::HandleVelocityStatus, this);
  sub->setQueueCapacity(1);
  while(true) {
    if (lcm_->handleTimeout(10) < 0) {
      break;
    }
    if (!is_running_) {
      break;
    }    
  }
}

void AdmittanceTrajectoryPlan::SubscribePose() {
  auto sub = lcm_->subscribe("RELATIVE_POSE",
    &AdmittanceTrajectoryPlan::HandlePoseStatus, this);
  sub->setQueueCapacity(1);
  while(true) {
    if (lcm_->handleTimeout(10) < 0) {
      break;
    }
    if (!is_running_) {
      break;
    }    
  }
}

void AdmittanceTrajectoryPlan::HandleForceTorqueStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    F_TC_ = data_eigen.cast<double>();
}      


void AdmittanceTrajectoryPlan::HandleVelocityStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    V_TC_ = data_eigen.cast<double>();    
}

void AdmittanceTrajectoryPlan::HandlePoseStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    const auto X_TC = data_eigen.cast<double>();
    const Eigen::Quaterniond q_TC(X_TC[0], X_TC[1], X_TC[2], X_TC[3]);
    X_TC_ = drake::math::RigidTransformd(q_TC, X_TC.tail(3));
}
