#include "state_error.h"

using std::cout;
using std::endl;

PlanManagerStateBase* StateError::instance_ = nullptr;
PlanManagerStateBase* StateError::Instance() {
  if (!instance_) {
    instance_ = new StateError();
  }
  return instance_;
}

void StateError::PrintCurrentState(const PlanManagerStateMachine *state_machine)
const {
  cout << "[ERROR]: "
       << "Number of plans: "  << state_machine->num_plans() << "."
       << endl;
}