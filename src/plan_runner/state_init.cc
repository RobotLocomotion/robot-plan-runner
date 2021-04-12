#include "state_init.h"
#include "state_idle.h"

using std::cout;
using std::endl;

PlanManagerStateBase* StateInit::instance_ = nullptr;
PlanManagerStateBase* StateInit::Instance() {
  if (!instance_) {
    instance_ = new StateInit();
  }
  return instance_;
}

const PlanBase* StateInit::get_current_plan(const PlanManagerStateMachine*
state_machine) const {
  DRAKE_THROW_UNLESS(state_machine->num_plans() == 0);
  return nullptr;
}

void StateInit::receive_new_status_msg(PlanManagerStateMachine *state_machine)
const {
  ChangeState(state_machine, StateIdle::Instance());
}

void StateInit::QueueNewPlan(PlanManagerStateMachine *state_machine,
                             std::shared_ptr<PlanBase> plan) {
  std::cout << "[RobotPlanRunner]: no robot status message received yet. "
               "Plan is not queued." << std::endl;
}

void StateInit::PrintCurrentState(const PlanManagerStateMachine *state_machine)
const {
  cout << "[INIT ]: waiting for IIWA_STATUS. "
       << "Number of plans: "  << state_machine->num_plans() << "."
       << endl;
}
