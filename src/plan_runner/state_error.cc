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

const PlanBase * StateError::get_current_plan(const PlanManagerStateMachine *
state_machine) const {
  throw std::runtime_error("this function should not be called in state ERROR");
}

void StateError::receive_new_status_msg(PlanManagerStateMachine *state_machine) const {
  throw std::runtime_error("this function should not be called in state ERROR");
}

void StateError::QueueNewPlan(PlanManagerStateMachine *state_machine,
                              std::shared_ptr<PlanBase> plan) {
  throw std::runtime_error("this function should not be called in state ERROR");
}

void StateError::PrintCurrentState(const PlanManagerStateMachine *state_machine) const {
  cout << "[ERROR]: "
       << "Number of plans: "  << state_machine->num_plans() << "."
       << endl;
}