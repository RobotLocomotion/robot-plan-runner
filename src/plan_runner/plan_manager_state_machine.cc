#include "plan_manager_state_machine.h"
#include "state_init.h"

PlanManagerStateMachine::PlanManagerStateMachine() {
  // Initialize to state INIT.
  state_ = StateInit::Instance();
}

void PlanManagerStateMachine::ChangeState(PlanManagerStateBase *new_state) {
  state_ = new_state;
}

const PlanBase* PlanManagerStateMachine::get_current_plan() const {
  return state_->get_current_plan(this);
}

double PlanManagerStateMachine::get_plan_time() const {
  return state_->get_plan_time();
}

bool PlanManagerStateMachine::has_received_status_msg() const {
  return state_->has_received_status_msg();
}

void PlanManagerStateMachine::receive_new_status_msg(
    PlanManagerStateMachine *manager) const {
  state_->receive_new_status_msg(manager);
}

void PlanManagerStateMachine::QueueNewPlan(std::shared_ptr<PlanBase> plan) const {
  state_->QueueNewPlan(plan);
}

bool PlanManagerStateBase::CheckCommandForError(const Command &cmd) const {
  return false;
}