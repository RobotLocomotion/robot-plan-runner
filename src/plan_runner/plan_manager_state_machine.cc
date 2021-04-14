#include "plan_manager_state_machine.h"
#include "state_init.h"

using std::string;

PlanManagerStateMachine::PlanManagerStateMachine() {
  // Initialize to state INIT.
  state_ = StateInit::Instance();
}

double PlanManagerStateMachine::get_current_plan_up_time(
    const std::chrono::time_point<std::chrono::high_resolution_clock>& t_now
    ) const {
  if (current_plan_start_time_ == nullptr) {
    return -1;
  }

  std::chrono::duration<double> t_elapsed_duration =
      t_now - *current_plan_start_time_;
  return t_elapsed_duration.count();
}

void PlanManagerStateMachine::set_current_plan_start_time(
    const std::chrono::time_point<std::chrono::high_resolution_clock>& t) {
  current_plan_start_time_ = std::make_unique<
      std::chrono::time_point<std::chrono::high_resolution_clock>>(t);
}

bool PlanManagerStateMachine::CommandHasError(const State &state,
                                              const Command &cmd) {
  return state_->CommandHasError(state, cmd, this);
}

void PlanManagerStateBase::receive_new_status_msg(
    PlanManagerStateMachine *) const {
  string error_msg = "receive_new_status_msg should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

void PlanManagerStateBase::QueueNewPlan(PlanManagerStateMachine *,
                                        std::shared_ptr<PlanBase>) {
  string error_msg = "QueueNewPlan should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

bool PlanManagerStateBase::CommandHasError(const State &, const Command &,
                                           PlanManagerStateMachine *) {
  string error_msg = "CommandHasError should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

double PlanManagerStateBase::get_current_plan_up_time(
    const PlanManagerStateMachine *state_machine) const {
  string error_msg = "get_current_plan_up_time should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

bool PlanManagerStateBase::has_received_status_msg() const {
  string error_msg = "has_received_status_msg should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

