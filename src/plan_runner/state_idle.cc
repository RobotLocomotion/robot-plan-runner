#include "state_idle.h"
#include "state_running.h"

PlanManagerStateBase* StateIdle::instance_ = nullptr;
PlanManagerStateBase* StateIdle::Instance() {
  if (!instance_) {
    instance_ = new StateIdle();
  }
  return instance_;
}

const PlanBase * StateIdle::get_current_plan(const PlanManagerStateMachine*
state_machine) const {
  const auto& plans_queue = state_machine->get_plans_queue();
  DRAKE_THROW_UNLESS(plans_queue.empty());
  return nullptr;
}

void StateIdle::QueueNewPlan(PlanManagerStateMachine *state_machine,
                                         std::shared_ptr<PlanBase> plan) {
  auto& plan_queue = state_machine->get_mutable_plans_queue();
  plan_queue.push(plan);
  ChangeState(state_machine, StateRunning::Instance());
}
