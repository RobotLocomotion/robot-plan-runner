#include "state_idle.h"
#include "state_running.h"

using std::cout;
using std::endl;

PlanManagerStateBase* StateIdle::instance_ = nullptr;
PlanManagerStateBase* StateIdle::Instance() {
  if (!instance_) {
    instance_ = new StateIdle();
  }
  return instance_;
}

const PlanBase * StateIdle::GetCurrentPlan(PlanManagerStateMachine *state_machine,
                                           double t_now) const {
  DRAKE_THROW_UNLESS(state_machine->num_plans() == 0);
  return nullptr;
}

void StateIdle::QueueNewPlan(PlanManagerStateMachine *state_machine,
                             std::unique_ptr<PlanBase> plan) {
  auto& plan_queue = state_machine->get_mutable_plans_queue();
  plan_queue.push(std::move(plan));
  ChangeState(state_machine, StateRunning::Instance());
}

std::string StateIdle::PrintCurrentState(
    const PlanManagerStateMachine *state_machine) const {
  std::string msg("[IDLE]: waiting for new plans. ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ".";
  cout << msg << endl;
  return std::move(msg);
}


