#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"

#include "hybrid_trajectory_plan.h"

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

void HybridTrajectoryPlan::Step(const State &state, double control_period,
                                   double t, Command *cmd) const {

  // 1. Update diffik mbp with the current status of the robot.
  Eigen::Matrix<double, 14, 1> qv;
  qv << state.q, state.v;
  plant_->SetPositionsAndVelocities(plant_context_.get(), qv);

  // 2. Compute current position and desired transforms.
  const drake::math::RigidTransformd X_WT_desired(quat_traj_.orientation(t),
                                            xyz_traj_.value(t));
  const auto& frame_W = plant_->world_frame();
  const auto X_WE = plant_->CalcRelativeTransform(
      *plant_context_, frame_W, frame_E_);
  const auto X_WT = X_WE * X_ET_;

  // 3. Compute current velocity.
  const auto V_WE = plant_->EvalBodySpatialVelocityInWorld(
    *plant_context_, frame_E_.body());
  const auto p_ET_W = X_WE.rotation().matrix() * X_ET_.translation();
  const auto V_WT = V_WE.Shift(p_ET_W);

  // 3. Compute the translational part of correction.
  Eigen::Vector3d p_WT_d = X_WT_desired.translation();
  // We will leave the x,y coordinates of p_WT_d alone and modify the 
  // z_part. The z part is computed by X_WT * X_TC * Xd_CT 
  // where Xd_TC satisfies the bushing equation Fzd = Kz * zx_TC

  Eigen::Vector3d fd_T = X_WT.inverse() * fd_W_;
  auto Kxyz_mat = Eigen::Matrix3d(Kxyz_.asDiagonal());
  auto xd_TC = drake::math::RigidTransformd(Kxyz_mat.inverse() * fd_T);

  auto Xd_WT = X_WT * X_TC_ * xd_TC.inverse();
  
  Eigen::Vector3d p_WT_corrected;
  p_WT_corrected[0] = p_WT_d[0];
  p_WT_corrected[1] = p_WT_d[1];
  p_WT_corrected[2] = Xd_WT.translation()[2]; // projection.

  std::cout << X_WT.translation()[2] << std::endl;
  std::cout << p_WT_corrected[2] << std::endl;
  std::cout << "===============" << std::endl;

  // 4. Compute the rotational part of correction.
  double yd_TC = 0.0; // the desired yaw angle is zero.
  double pd_TC = (1./Krpy_[1]) * (taud_T_[1] * cos(yd_TC) - taud_T_[0] * sin(yd_TC));
  double rd_TC = (1./Krpy_[0]) * ((taud_T_[0] * cos(yd_TC) + taud_T_[1] * sin(yd_TC)
                          ) * cos(pd_TC) - Krpy_[1] * yd_TC * sin(pd_TC));

  drake::math::RollPitchYaw<double> rpyd_TC(rd_TC, pd_TC, yd_TC);
  drake::math::RotationMatrixd Rd_TC(rpyd_TC);

  auto R_WT_corrected = X_WT.rotation() * X_TC_.rotation() * Rd_TC.inverse();
  drake::math::RigidTransformd X_WT_corrected(R_WT_corrected, p_WT_corrected);

  // A factor of 0.1 is multiplied because X_WT_corrected is updated from 
  // relative pose, which is published at ~20Hz. Since the robot is sending
  // q_cmd at 200Hz, we apply a zero-order hold this way.

  Vector6<double> V_WT_desired =
      ComputePoseDiffInCommonFrame(
          X_WT.GetAsIsometry3(), X_WT_corrected.GetAsIsometry3()) /
          params_->get_timestep();

  double kp = 0.03;
  double kd = 0.1;

  Vector6<double> V_WT_corrected = kp * V_WT_desired - kd * V_WT.get_coeffs();
  
  MatrixX<double> J_WT(6, plant_->num_velocities());
  plant_->CalcJacobianSpatialVelocity(*plant_context_,
                                    drake::multibody::JacobianWrtVariable::kV,
                                    frame_E_, X_ET_.translation(),
                                    frame_W, frame_W, &J_WT);


  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      state.q, state.v, X_WT, J_WT,
      drake::multibody::SpatialVelocity<double>(V_WT_corrected), *params_);

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

HybridTrajectoryPlan::~HybridTrajectoryPlan() {
  is_running_ = false;
  // Wait for all threads to terminate.
  for (auto &a : threads_) {
    if (a.second.joinable()) {
      a.second.join();
    }
  }
}

void HybridTrajectoryPlan::SubscribeForceTorque() {
  auto sub = lcm_->subscribe("FT",
    &HybridTrajectoryPlan::HandleForceTorqueStatus, this);
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

void HybridTrajectoryPlan::SubscribeVelocity() {
  auto sub = lcm_->subscribe("RELATIVE_VELOCITY",
    &HybridTrajectoryPlan::HandleVelocityStatus, this);
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

void HybridTrajectoryPlan::SubscribePose() {
  auto sub = lcm_->subscribe("RELATIVE_POSE",
    &HybridTrajectoryPlan::HandlePoseStatus, this);
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

void HybridTrajectoryPlan::HandleForceTorqueStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    F_TC_ = data_eigen.cast<double>();
}      


void HybridTrajectoryPlan::HandleVelocityStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    V_TC_ = data_eigen.cast<double>();    
}

void HybridTrajectoryPlan::HandlePoseStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    const auto X_TC = data_eigen.cast<double>();
    const Eigen::Quaterniond q_TC(X_TC[0], X_TC[1], X_TC[2], X_TC[3]);
    X_TC_ = drake::math::RigidTransformd(q_TC, X_TC.tail(3));
}
