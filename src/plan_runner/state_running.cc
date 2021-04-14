#include "state_idle.h"
#include "state_running.h"
#include "state_error.h"

using std::cout;
using std::endl;

PlanManagerStateBase* StateRunning::instance_ = nullptr;
PlanManagerStateBase* StateRunning::Instance() {
  if (!instance_) {
    instance_ = new StateRunning();
  }
  return instance_;
}

const PlanBase * StateRunning::GetCurrentPlan(
    PlanManagerStateMachine *state_machine) const {
  auto& plans = state_machine->get_mutable_plans_queue();
  DRAKE_THROW_UNLESS(!plans.empty());

  auto t_now = std::chrono::high_resolution_clock::now();
  const double t_elapsed_seconds =
      state_machine->get_current_plan_up_time(t_now);
  if (t_elapsed_seconds < 0) {
    // current_plan_start_time_ is nullptr, meaning that there is no active
    // plan.
    state_machine->set_current_plan_start_time(t_now);
  } else if (t_elapsed_seconds > plans.front()->duration() + 1.0) {
    // current plan has expired, switching to the next plan.
    state_machine->set_current_plan_start_time(t_now);
    plans.pop();
  }
  if (plans.empty()) {
    state_machine->reset_current_plan_start_time();
    ChangeState(state_machine, StateIdle::Instance());
    return nullptr;
  }
  return plans.front().get();
}

double StateRunning::get_current_plan_up_time(
    const PlanManagerStateMachine *state_machine) const {
  auto t_now = std::chrono::high_resolution_clock::now();
  return state_machine->get_current_plan_up_time(t_now);
}

PlanExecutionStatus StateRunning::get_plan_execution_status() const {
  throw std::runtime_error("not implemented yet.");
}

void StateRunning::QueueNewPlan(PlanManagerStateMachine *state_machine,
                                std::shared_ptr<PlanBase> plan) {
  //TODO: what is the desired behavior when receiving a new plan while a plan
  // is still being executed? Discard or queue?
  // Discard is probably the better option? The client should be blocked
  // while a plan is executing. A vector of plans can be sent over LCM to
  // be queued when state is IDLE.
  std::cout << "[Running]: another plan is running. "
               "Received plan is discarded."
            << std::endl;
}

// return true: has error; false: has no error.
bool StateRunning::CommandHasError(const State &state,
                                   const Command &cmd,
                                   PlanManagerStateMachine *state_machine) {
  bool is_nan =
      cmd.q_cmd.array().isNaN().sum() or cmd.tau_cmd.array().isNaN().sum();

  bool is_too_far_away = (state.q - cmd.q_cmd).norm() > 0.05;

  bool is_error = is_nan or is_too_far_away;
  ChangeState(state_machine, StateError::Instance());
  return is_error;
}

void StateRunning::PrintCurrentState(
    const PlanManagerStateMachine *state_machine) const {
  cout << "[RUNNING]: executing a plan. "
       << "Number of plans: " << state_machine->num_plans() << "." << endl;
}

