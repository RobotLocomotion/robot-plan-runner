#include "plan_manager.h"
#include "plan_base.h"

#include "drake_lcmtypes/drake/lcmt_iiwa_command.hpp"

using Eigen::VectorXd;

IiwaPlanManager::IiwaPlanManager(double control_period)
    : control_period_(control_period) {
  state_machine_ = std::make_unique<PlanManagerStateMachine>();
}

IiwaPlanManager::~IiwaPlanManager() {
  // Terminate all threads.
  for (auto &a : threads_) {
    if (a.second.joinable()) {
      a.second.join();
    }
  }
}

void IiwaPlanManager::CalcCommandFromStatus() {
  lcm_status_command_ = std::make_unique<lcm::LCM>();
  lcm_status_command_->subscribe("IIWA_STUATS", &IiwaPlanManager::HandleIiwaStatus,
                                 this);
  // Call lcm handle until at least one status message is processed.
  while (0 == lcm_status_command_->handleTimeout(10) ||
         !state_machine_->has_received_status_msg()) {}

}

void IiwaPlanManager::Run() {
  threads_["status_command_thread"] = std::thread(
      &IiwaPlanManager::CalcCommandFromStatus, this);
}

void IiwaPlanManager::HandleIiwaStatus(
    const lcm::ReceiveBuffer *, const std::string &channel,
    const drake::lcmt_iiwa_status *status_msg) {
  {
    std::lock_guard<std::mutex> lock(mutex_iiwa_status_);
    iiwa_status_msg_ = *status_msg;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_state_machine_);
    state_machine_->receive_new_status_msg();
  }
  State s(Eigen::Map<VectorXd>(iiwa_status_msg_.joint_position_measured.data(),
                               iiwa_status_msg_.num_joints),
          Eigen::Map<VectorXd>(iiwa_status_msg_.joint_position_measured.data(),
                               iiwa_status_msg_.num_joints),
          Eigen::Map<VectorXd>(iiwa_status_msg_.joint_torque_external.data(),
                               iiwa_status_msg_.num_joints));
  Command c;
  const PlanBase* plan = state_machine_->GetCurrentPlan();
  if (plan) {
    plan ->Step(s, control_period_,
                state_machine_->get_current_plan_up_time(), &c);
  } else {
    //TODO: no command is sent if there is no plan. Make sure that this is
    // the desired behavior.
    return;
  }

  if (!state_machine_->CommandHasError(s, c)) {
    drake::lcmt_iiwa_command cmd_msg;
    cmd_msg.num_joints = status_msg->num_joints;
    cmd_msg.utime = status_msg->utime;
    for (int i = 0; i < cmd_msg.num_joints; i++) {
      cmd_msg.joint_position.push_back(c.q_cmd[i]);
      cmd_msg.joint_torque.push_back(c.tau_cmd[i]);
    }
    lcm_status_command_->publish("IIWA_COMMAND", &cmd_msg);
  }
}
