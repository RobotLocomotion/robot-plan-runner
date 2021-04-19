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

const PlanBase *StateInit::GetCurrentPlan(PlanManagerStateMachine *state_machine,
                                          double t_now) const {
  DRAKE_THROW_UNLESS(state_machine->num_plans() == 0);
  return nullptr;
}

void StateInit::receive_new_status_msg(
    PlanManagerStateMachine *state_machine) const {
  ChangeState(state_machine, StateIdle::Instance());
}

void StateInit::QueueNewPlan(PlanManagerStateMachine *state_machine,
                             std::unique_ptr<PlanBase> plan) {
  std::cout << "[INIT]: no robot status message received yet. "
               "Received plan is discarded."
            << std::endl;
}

std::string StateInit::PrintCurrentState(
    const PlanManagerStateMachine *state_machine) const {
  std::string msg("[INIT]: waiting for IIWA_STATUS. ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ".";
  cout << msg << endl;
  return std::move(msg);
}
