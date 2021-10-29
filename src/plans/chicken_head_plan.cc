#include "plans/chicken_head_plan.h"
#include "drake/lcm/lcm_messages.h"

using drake::Vector6;
using drake::math::RigidTransformd;
using drake::lcmt_robot_state;
using drake::manipulation::planner::internal::DoDifferentialInverseKinematics;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;

ChickenHeadPlan::ChickenHeadPlan(
    const drake::math::RigidTransformd &X_WL7_0, double duration,
    double control_time_step,
    const Eigen::Ref<const Eigen::VectorXd> &nominal_joint_angles,
    const drake::multibody::MultibodyPlant<double> *plant)
    : PlanBase(plant), duration_(duration), X_WCd_(X_WL7_0 * X_L7T_),
      frame_L7_(plant->GetFrameByName("iiwa_link_7")) {
  // DiffIk.
  params_ = std::make_unique<
      drake::manipulation::planner::DifferentialInverseKinematicsParameters>(
      plant_->num_positions(), plant_->num_velocities());
  params_->set_timestep(control_time_step);
  params_->set_nominal_joint_position(nominal_joint_angles);
  plant_context_ = plant_->CreateDefaultContext();

  // X_TC subscription.
  owned_lcm_ = std::make_unique<drake::lcm::DrakeLcm>();
  rpe_sub_ = std::make_unique<drake::lcm::Subscriber<drake::lcmt_robot_state>>(
      owned_lcm_.get(), "X_TC");
  sub_thread_ = std::thread(&ChickenHeadPlan::PoseIo, this);
}

ChickenHeadPlan::~ChickenHeadPlan() {
  stop_flag_ = true;
  sub_thread_.join();
}

lcmt_robot_state GetStateMsgFromTransform(
    const RigidTransformd& X_WC, int64_t utime) {
  const auto Q_WC = X_WC.rotation().ToQuaternion();
  const auto& p_WC = X_WC.translation();
  lcmt_robot_state msg_X_WC{};
  msg_X_WC.utime = utime;
  msg_X_WC.num_joints = 7;
  msg_X_WC.joint_name = {"qw", "qx", "qy", "qz", "x", "y", "z"};
  msg_X_WC.joint_position = std::vector<float>(
      {static_cast<float>(Q_WC.w()),
       static_cast<float>(Q_WC.x()),
       static_cast<float>(Q_WC.y()),
       static_cast<float>(Q_WC.z()),
       static_cast<float>(p_WC[0]),
       static_cast<float>(p_WC[1]),
       static_cast<float>(p_WC[2])});
  return msg_X_WC;
}

void ChickenHeadPlan::PoseIo() const {
  while (true) {
    rpe_sub_->clear();
    drake::lcm::LcmHandleSubscriptionsUntil(
        owned_lcm_.get(),
        [&]() { return stop_flag_ or rpe_sub_->count() > 0; });
    if (stop_flag_) {
      break;
    }
    const auto &X_TC_msg = rpe_sub_->message();
    const auto &q_xyz = X_TC_msg.joint_position;
    const auto Q_TC =
        Eigen::Quaterniond(q_xyz[0], q_xyz[1], q_xyz[2], q_xyz[3]);
    const auto p_TC = Eigen::Vector3d(q_xyz[4], q_xyz[5], q_xyz[6]);

    drake::math::RigidTransformd X_WC;
    {
      std::lock_guard<std::mutex> lock(mutex_rpe_);
      X_TC_.set_rotation(Q_TC);
      X_TC_.set_translation(p_TC);
      X_WC = X_WT_ * X_TC_;
    }

    // Publish X_WC.
    drake::lcm::Publish(
        owned_lcm_.get(), "X_WC", GetStateMsgFromTransform(X_WC, X_TC_msg.utime));
  }
}

void ChickenHeadPlan::Step(const State &state, double control_period, double t,
                           Command *cmd) const {
  // Forward kinematics.
  plant_->SetPositions(plant_context_.get(), state.q);
  const auto &frame_W = plant_->world_frame();
  const auto X_WT =
      plant_->CalcRelativeTransform(*plant_context_, frame_W, frame_L7_) *
      X_L7T_;

  // New T_desired.
  RigidTransformd X_WTd;
  {
    std::lock_guard<std::mutex> lock(mutex_rpe_);
    X_WTd = X_WCd_ * X_TC_.inverse();
    // Also Update X_WT_.
    X_WT_ = X_WT;
  }

  // DiffIk.
  const Vector6<double> V_WT_desired =
      drake::manipulation::planner::ComputePoseDiffInCommonFrame(
          X_WT.GetAsIsometry3(), X_WTd.GetAsIsometry3()) /
      params_->get_timestep();

  Eigen::MatrixXd J_WT(6, plant_->num_velocities());
  plant_->CalcJacobianSpatialVelocity(
      *plant_context_, drake::multibody::JacobianWrtVariable::kV, frame_L7_,
      X_L7T_.translation(), frame_W, frame_W, &J_WT);

  const auto result = DoDifferentialInverseKinematics(
      state.q, state.v, X_WT, J_WT,
      drake::multibody::SpatialVelocity<double>(V_WT_desired), *params_);

  // 3. Check for errors and integrate.
  if (result.status != DifferentialInverseKinematicsStatus::kSolutionFound) {
    cmd->q_cmd = NAN * Eigen::VectorXd::Zero(7);
    spdlog::critical("DoDifferentialKinematics Failed to find a solution.");
  } else {
    cmd->q_cmd = state.q + control_period * result.joint_velocities.value();
    cmd->tau_cmd = Eigen::VectorXd::Zero(7);
  }

  // Publish X_WC.
}
