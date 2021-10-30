#include "plans/screwdriver_plan.h"


using drake::lcmt_robot_state;
using drake::Vector6;
using drake::manipulation::planner::ComputePoseDiffInCommonFrame;
using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::manipulation::planner::internal::DoDifferentialInverseKinematics;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using Eigen::Vector3d;
using Eigen::VectorXd;

ScrewdriverPlan::ScrewdriverPlan(
    drake::trajectories::PiecewisePolynomial<double> q_traj,
    const drake::multibody::MultibodyPlant<double> *plant,
    double control_time_step)
    : PlanBase(plant), q_traj_(std::move(q_traj)) {

  // DiffIk.
  plant_context_ = plant_->CreateDefaultContext();

  // X_TC subscription.
  owned_lcm_ = std::make_unique<drake::lcm::DrakeLcm>();
  rpe_sub_ = std::make_unique<drake::lcm::Subscriber<drake::lcmt_robot_state>>(
      owned_lcm_.get(), "X_TC");
  sub_thread_ = std::thread(&ScrewdriverPlan::PoseIo, this);

  yaw_TC_desired_ = 0.05;
  yaw_TC_now_ = 0.05;
  q_set_ = 0.0;
  controller_active_ = false;
}

ScrewdriverPlan::~ScrewdriverPlan() {
  stop_flag_ = true;
  sub_thread_.join();
}

lcmt_robot_state GetStateMsgFromTransform(const RigidTransformd &X_WC,
                                          int64_t utime) {
  const auto Q_WC = X_WC.rotation().ToQuaternion();
  const auto &p_WC = X_WC.translation();
  lcmt_robot_state msg_X_WC{};
  msg_X_WC.utime = utime;
  msg_X_WC.num_joints = 7;
  msg_X_WC.joint_name = {"qw", "qx", "qy", "qz", "x", "y", "z"};
  msg_X_WC.joint_position = std::vector<float>(
      {static_cast<float>(Q_WC.w()), static_cast<float>(Q_WC.x()),
       static_cast<float>(Q_WC.y()), static_cast<float>(Q_WC.z()),
       static_cast<float>(p_WC[0]), static_cast<float>(p_WC[1]),
       static_cast<float>(p_WC[2])});
  return msg_X_WC;
}

void ScrewdriverPlan::PoseIo() const {
  while (true) {
    rpe_sub_->clear();
    drake::lcm::LcmHandleSubscriptionsUntil(owned_lcm_.get(), [&]() {
      return stop_flag_ or rpe_sub_->count() > 0;
    }, 10);
    if (stop_flag_) {
      break;
    }
    const auto &X_TC_msg = rpe_sub_->message();
    const auto &q_xyz = X_TC_msg.joint_position;
    const auto Q_TC =
        Eigen::Quaterniond(q_xyz[0], q_xyz[1], q_xyz[2], q_xyz[3]);
    const auto p_TC = Eigen::Vector3d(q_xyz[4], q_xyz[5], q_xyz[6]);

    const auto rpy_TC = drake::math::RollPitchYaw<double>(Q_TC).vector();
    yaw_TC_now_ = rpy_TC[2];

    drake::math::RigidTransformd X_WC;
    {
      std::lock_guard<std::mutex> lock(mutex_rpe_);
      X_TC_.set_rotation(Q_TC);
      X_TC_.set_translation(p_TC);
      X_WC = X_WT_ * X_TC_;
    }

    // Publish X_WC.
    drake::lcm::Publish(owned_lcm_.get(), "X_WC",
                        GetStateMsgFromTransform(X_WC, X_TC_msg.utime));
  }
}

void ScrewdriverPlan::Step(const State &state, double control_period, double t,
                           Command *cmd) const {

  Eigen::VectorXd q_desired = q_traj_.value(t);
  std::cout << q_desired << std::endl;

  if ((yaw_TC_now_ >= yaw_TC_desired_) && (!controller_active_)) {
    controller_active_ = true;
    q_set_ = state.q[6];
  }

  if (controller_active_) {
    q_desired[6] = q_set_;
  }

  // 3. Check for errors and integrate.
  cmd->q_cmd = q_desired;
  cmd->tau_cmd = Eigen::VectorXd::Zero(7);
}
