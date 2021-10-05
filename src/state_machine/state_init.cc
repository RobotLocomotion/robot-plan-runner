#include "state_init.h"
#include "state_idle.h"

using std::cout;
using std::endl;

PlanManagerStateBase *StateInit::instance_ = nullptr;
PlanManagerStateBase *StateInit::Instance() {
  if (!instance_) {
    instance_ = new StateInit();
  }
  return instance_;
}

void StateInit::ReceiveNewStatusMsg(
    PlanManagerStateMachine *state_machine,
    const drake::lcmt_iiwa_status &msg_iiwa_status) const {
  state_machine->SetIiwaPositionCommandIdle(Eigen::Map<const Eigen::VectorXd>(
      msg_iiwa_status.joint_position_commanded.data(),
      msg_iiwa_status.num_joints));
  ChangeState(state_machine, StateIdle::Instance());
}

void StateInit::QueueNewPlan(PlanManagerStateMachine *state_machine,
                             std::unique_ptr<PlanBase> plan) {
  spdlog::warn("[INIT]: no robot status message received yet. "
               "Received plan is discarded.");
}

bool StateInit::CommandHasError(const State &state, const Command &cmd,
                                PlanManagerStateMachine *state_machine,
                                const double q_threshold) {
  std::string error_msg = "CommandHasError should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

void StateInit::PrintCurrentState(const PlanManagerStateMachine *state_machine,
                                  double t_now_seconds) const {
  std::string msg;
  msg += ("[INIT]: waiting for IIWA_STATUS. ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ". t = ";
  msg +=
      std::to_string(state_machine->get_state_machine_up_time(t_now_seconds));
  spdlog::info(msg);
}
