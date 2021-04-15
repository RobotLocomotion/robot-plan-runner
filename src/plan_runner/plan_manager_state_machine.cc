#include "plan_manager_state_machine.h"
#include "state_init.h"

using std::string;

PlanManagerStateMachine::PlanManagerStateMachine() {
  // Initialize to state INIT.
  state_ = StateInit::Instance();
}

double PlanManagerStateBase::GetCurrentPlanUpTime(
    const PlanManagerStateMachine *state_machine,
    const TimePoint &t_now) const {
  string error_msg = "GetCurrentPlanUpTime should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

void PlanManagerStateBase::receive_new_status_msg(
    PlanManagerStateMachine *) const {
  string error_msg = "receive_new_status_msg should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

void PlanManagerStateBase::QueueNewPlan(PlanManagerStateMachine *state_machine,
                                        std::unique_ptr<PlanBase> plan) {
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

bool PlanManagerStateBase::has_received_status_msg() const {
  string error_msg = "has_received_status_msg should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}
