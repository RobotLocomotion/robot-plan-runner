#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"

#include "rigid_trajectory_plan.h"

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

void RigidTrajectoryPlan::Step(const State &state, double control_period,
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

  Eigen::Matrix<double, 6, 1> V_T;
  V_T << X_WT.inverse().rotation().matrix() * V_WT.rotational(),
          X_WT.inverse().rotation().matrix() * V_WT.translational();

  // 3. Compute the translational part of correction.
  Eigen::Matrix<double, 6, 1> F_TC_W;
  auto tau_TC_W = X_WT.rotation().matrix() * F_TC_.head(3);
  auto f_TC_W = X_WT.rotation().matrix() * F_TC_.tail(3);
  F_TC_W << tau_TC_W, f_TC_W;
    // Compare fd_W with current force to produce desired velocity.
  
  double kp_vz = 0.001;
  double kd_vz = 0.1;
  double v_WT_z = - kp_vz * (F_TC_W(5) - fd_W_(2)) - kd_vz * V_WT.translational()[2];


  // 4. Compute the rotational part of correction.
  double kp_wx = 0.001;
  double kd_wx = 0.1;  
  double w_WT_x = -kp_wx * F_TC_W(0) - kd_wx * V_WT.rotational()[0];

  Vector6<double> V_WT_desired =
      ComputePoseDiffInCommonFrame(
          X_WT.GetAsIsometry3(), X_WT_desired.GetAsIsometry3()) /
          params_->get_timestep();

  V_WT_desired(0) = w_WT_x;
  V_WT_desired(5) = v_WT_z;

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

RigidTrajectoryPlan::~RigidTrajectoryPlan() {
  is_running_ = false;
  // Wait for all threads to terminate.
  for (auto &a : threads_) {
    if (a.second.joinable()) {
      a.second.join();
    }
  }
}

void RigidTrajectoryPlan::SubscribeForceTorque() {
  auto sub = lcm_->subscribe("FT",
    &RigidTrajectoryPlan::HandleForceTorqueStatus, this);
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

void RigidTrajectoryPlan::SubscribeVelocity() {
  auto sub = lcm_->subscribe("RELATIVE_VELOCITY",
    &RigidTrajectoryPlan::HandleVelocityStatus, this);
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

void RigidTrajectoryPlan::SubscribePose() {
  auto sub = lcm_->subscribe("RELATIVE_POSE",
    &RigidTrajectoryPlan::HandlePoseStatus, this);
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

void RigidTrajectoryPlan::HandleForceTorqueStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    F_TC_ = data_eigen.cast<double>();
}      


void RigidTrajectoryPlan::HandleVelocityStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    V_TC_ = data_eigen.cast<double>();    
}

void RigidTrajectoryPlan::HandlePoseStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    const auto X_TC = data_eigen.cast<double>();
    const Eigen::Quaterniond q_TC(X_TC[0], X_TC[1], X_TC[2], X_TC[3]);
    X_TC_ = drake::math::RigidTransformd(q_TC, X_TC.tail(3));
}
