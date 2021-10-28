#include "plans/chicken_head_plan.h"



ChickenHeadPlan::ChickenHeadPlan(
    const drake::math::RigidTransformd& X_WL7_0,
    const drake::multibody::MultibodyPlant<double> *plant)
    : PlanBase(plant), X_WCd_(X_WL7_0 * X_L7T_),
      frame_L7_(plant->GetFrameByName("iiwa_link_7")) {
  // X_TC subscription.
  owned_lcm_ = std::make_unique<drake::lcm::DrakeLcm>();
  rpe_sub_ = std::make_unique<drake::lcm::Subscriber<drake::lcmt_robot_state>>(
      owned_lcm_.get(), "X_TC");
  sub_thread_ = std::thread(&ChickenHeadPlan::ReceiveRelativePose, this);
}

ChickenHeadPlan::~ChickenHeadPlan() {
  if (sub_thread_.joinable()) {
    sub_thread_.join();
  }
}

[[noreturn]] void ChickenHeadPlan::ReceiveRelativePose() const {
  while (true) {
    rpe_sub_->clear();
    drake::lcm::LcmHandleSubscriptionsUntil(
        owned_lcm_.get(), [&]() { return rpe_sub_->count() > 0; });
    const auto& X_TC_msg = rpe_sub_->message();
    const auto& q_xyz = X_TC_msg.joint_position;
    const auto Q_TC = Eigen::Quaterniond(q_xyz[0], q_xyz[1], q_xyz[2],
                                         q_xyz[3]);
    const auto p_TC = Eigen::Vector3d(q_xyz[4], q_xyz[5], q_xyz[6]);
    {
      std::lock_guard<std::mutex> lock(mutex_rpe_);
      X_TC_.set_rotation(Q_TC);
      X_TC_.set_translation(p_TC);
    }
  }
}

void ChickenHeadPlan::Step(const State &state, double control_period, double t,
          Command *cmd) const {

}