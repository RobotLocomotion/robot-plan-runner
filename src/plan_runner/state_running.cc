#include "state_idle.h"
#include "state_running.h"

PlanManagerStateBase* StateRunning::instance_ = nullptr;
PlanManagerStateBase* StateRunning::Instance() {
  if (!instance_) {
    instance_ = new StateRunning();
  }
  return instance_;
}

const PlanBase * StateRunning::get_current_plan(
    const PlanManagerStateMachine *state_machine) const {
  const auto& plans = state_machine->get_plans_queue();
  DRAKE_THROW_UNLESS(!plans.empty());
  return plans.front().get();
}

void StateRunning::QueueNewPlan(PlanManagerStateMachine *state_machine,
                                std::shared_ptr<PlanBase> plan) {
  //TODO: what is the desired behavior when receiving a new plan while a plan
  // is still being executed? Discard or queue?
  // Discard is probably the better option? The client should be blocked
  // while a plan is executing. A vector of plans can be sent over LCM to
  // be queued when state is IDLE.
}

// return true: has error; false: has no error.
bool StateRunning::CheckCommandForError(const Command &cmd, const State &
state) const {
  bool is_nan =
      cmd.q_cmd.array().isNaN().sum() or cmd.tau_cmd.array().isNaN().sum();

  return is_nan;
}