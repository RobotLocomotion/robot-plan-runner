#include "state_machine/state_error.h"

#include <iostream>

using std::cout;
using std::endl;

PlanManagerStateBase *StateError::instance_ = nullptr;
PlanManagerStateBase *StateError::Instance() {
  if (!instance_) {
    instance_ = new StateError();
  }
  return instance_;
}

bool StateError::CommandHasError(const State &state, const Command &cmd,
                     PlanManagerStateMachine *state_machine,
                     const double q_threshold) {
  std::string error_msg = "CommandHasError should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

void StateError::PrintCurrentState(const PlanManagerStateMachine *state_machine,
                                   double t_now_seconds) const {
  std::string msg;
  msg += ("[ERROR]: ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ". t = ";
  msg +=
      std::to_string(state_machine->get_state_machine_up_time(t_now_seconds));
  spdlog::critical(msg);
}

void StateError::QueueNewPlan(PlanManagerStateMachine *state_machine,
                              std::unique_ptr<PlanBase> plan) {
  std::string msg("[ERROR]: received plan is discarded.");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ".";
  spdlog::critical(msg);
}
