#include "state_machine/state_error.h"

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
  std::string msg("t = ");
  msg +=
      std::to_string(state_machine->get_state_machine_up_time(t_now_seconds));
  msg += (". [ERROR]: ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ".";
  cout << msg << endl;
}

void StateError::QueueNewPlan(PlanManagerStateMachine *state_machine,
                              std::unique_ptr<PlanBase> plan) {
  cout << "[ERROR]: received plan is discarded."
       << "Number of plans: " << state_machine->num_plans() << "." << endl;
}
