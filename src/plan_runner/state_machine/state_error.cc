#include "state_error.h"

using std::cout;
using std::endl;

PlanManagerStateBase *StateError::instance_ = nullptr;
PlanManagerStateBase *StateError::Instance() {
  if (!instance_) {
    instance_ = new StateError();
  }
  return instance_;
}

std::string StateError::PrintCurrentState(
    const PlanManagerStateMachine *state_machine) const {
  std::string msg("[ERROR]: ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ".";
  cout << msg << endl;
  return std::move(msg);
}

void StateError::QueueNewPlan(PlanManagerStateMachine *state_machine,
                              std::unique_ptr<PlanBase> plan) {
  cout << "[ERROR]: received plan is discarded."
       << "Number of plans: " << state_machine->num_plans() << "." << endl;
}
