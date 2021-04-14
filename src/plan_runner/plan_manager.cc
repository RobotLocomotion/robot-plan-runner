#include "plan_manager.h"
#include "plans/plan_base.h"

#include "drake_lcmtypes/drake/lcmt_iiwa_command.hpp"

using Eigen::VectorXd;
using std::cout;
using std::endl;

IiwaPlanManager::IiwaPlanManager(double control_period)
    : control_period_(control_period) {
  state_machine_ = std::make_unique<PlanManagerStateMachine>();
}

IiwaPlanManager::~IiwaPlanManager() {
  // Wait for all threads to terminate.
  for (auto &a : threads_) {
    if (a.second.joinable()) {
      a.second.join();
    }
  }
}

void IiwaPlanManager::CalcCommandFromStatus() {
  lcm_status_command_ = std::make_unique<lcm::LCM>();
  lcm_status_command_->subscribe("IIWA_STATUS",
                                 &IiwaPlanManager::HandleIiwaStatus,
                                 this);
  while (true) {
    // >0 if a message was handled,
    // 0 if the function timed out,
    // <0 if an error occured.
    if (lcm_status_command_->handleTimeout(10) < 0) {
      break;
    }
  }
}

void IiwaPlanManager::PrintStateMachineStatus() const {
  using namespace std::chrono_literals;
  while (true) {
    std::this_thread::sleep_for(1s);
    {
      state_machine_->PrintCurrentState();
    }
  }
}

void IiwaPlanManager::Run() {
  threads_["status_command"] = std::thread(
      &IiwaPlanManager::CalcCommandFromStatus, this);
  threads_["print_status"] = std::thread(
      &IiwaPlanManager::PrintStateMachineStatus, this);
}

void IiwaPlanManager::HandleIiwaStatus(
    const lcm::ReceiveBuffer *, const std::string &channel,
    const drake::lcmt_iiwa_status *status_msg) {
  {
    std::lock_guard<std::mutex> lock(mutex_iiwa_status_);
    iiwa_status_msg_ = *status_msg;
  }
  const PlanBase* plan;
  {
    std::lock_guard<std::mutex> lock(mutex_state_machine_);
    state_machine_->receive_new_status_msg();
    plan = state_machine_->GetCurrentPlan();
  }

  // Compute command.
  State s(Eigen::Map<VectorXd>(iiwa_status_msg_.joint_position_measured.data(),
                               iiwa_status_msg_.num_joints),
          Eigen::Map<VectorXd>(iiwa_status_msg_.joint_position_measured.data(),
                               iiwa_status_msg_.num_joints),
          Eigen::Map<VectorXd>(iiwa_status_msg_.joint_torque_external.data(),
                               iiwa_status_msg_.num_joints));
  Command c;
  if (plan) {
    plan->Step(s, control_period_,
                state_machine_->get_current_plan_up_time(), &c);
  } else {
    //TODO: no command is sent if there is no plan. Make sure that this is
    // the desired behavior.
    return;
  }

  // Check command for error.
  bool command_has_error;
  {
    std::lock_guard<std::mutex> lock(mutex_state_machine_);
    command_has_error = state_machine_->CommandHasError(s, c);
  }
  if (!command_has_error) {
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
