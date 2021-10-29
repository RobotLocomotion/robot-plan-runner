#include "plans/chicken_head_plan.h"
#include "drake/lcm/lcm_messages.h"

using drake::lcmt_robot_state;
using drake::Vector6;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::manipulation::planner::internal::DoDifferentialInverseKinematics;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using Eigen::Vector3d;
using Eigen::VectorXd;

ChickenHeadPlan::ChickenHeadPlan(
    const drake::math::RigidTransformd &X_WL7_0, double duration,
    const drake::multibody::MultibodyPlant<double> *plant)
    : PlanBase(plant), duration_(duration), X_WCd_(X_WL7_0 * X_L7T_),
      frame_L7_(plant->GetFrameByName("iiwa_link_7")) {
  // DiffIk.
  plant_context_ = plant_->CreateDefaultContext();
  solver_ = std::make_unique<drake::solvers::GurobiSolver>();
  result_ = std::make_unique<drake::solvers::MathematicalProgramResult>();

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

void ChickenHeadPlan::PoseIo() const {
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

  const auto &R_WT = X_WT.rotation();
  const Vector3d e_xyz = X_WTd.translation() - X_WT.translation();
  const RotationMatrixd R_TTd = R_WT.inverse() * X_WTd.rotation();
  Vector6<double> V_WT_desired;
  V_WT_desired.head(3) =
      R_WT * (kp_rotation_ * R_TTd.ToQuaternion().vec().array()).matrix();
  V_WT_desired.tail(3) = kp_translation_ * e_xyz.array();

  plant_->CalcJacobianSpatialVelocity(
      *plant_context_, drake::multibody::JacobianWrtVariable::kV, frame_L7_,
      X_L7T_.translation(), frame_W, frame_W, &Jv_WTq_W_);

  const int nq = plant_->num_velocities();
  auto prog = drake::solvers::MathematicalProgram();
  auto dq = prog.NewContinuousVariables(nq, "dq");

  const Eigen::MatrixXd &A = Jv_WTq_W_;
  const Eigen::VectorXd b = V_WT_desired * control_period;
  // minimize ||A * dq - b||^2 + 0.01 * ||dq||^2

  prog.AddQuadraticCost(
      (A.transpose() * A + 0.01 * Eigen::MatrixXd::Identity(nq, nq)),
      -A.transpose() * b, dq);

  solver_->Solve(prog, {}, {}, result_.get());

  // 3. Check for errors and integrate.
  if (result_->is_success()) {
    cmd->q_cmd = state.q + result_->GetSolution(dq);
    cmd->tau_cmd = Eigen::VectorXd::Zero(nq);
  } else {
    throw DiffIkException();
  }
}
