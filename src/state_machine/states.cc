#include "states.h"

#include <iostream>
#include <spdlog/spdlog.h>

using std::cout;
using std::endl;

/* Init State */
PlanManagerStateBase *StateInit::instance_ = nullptr;
PlanManagerStateBase *StateInit::Instance() {
  if (!instance_) {
    instance_ = new StateInit();
  }
  return instance_;
}

void StateInit::ReceiveNewStatusMsg(PlanManagerStateMachine *state_machine,
                                    const State &state) const {
  state_machine->SetPositionCommandIdle(state.q);
  ChangeState(state_machine, StateIdle::Instance());
}

void StateInit::QueueNewPlan(PlanManagerStateMachine *state_machine,
                             std::unique_ptr<PlanBase> plan) {
  spdlog::warn("[INIT]: no robot status message received yet. "
               "Received plan is discarded.");
}

bool StateInit::CommandHasError(const State &state, const Command &cmd,
                                PlanManagerStateMachine *state_machine,
                                const double q_threshold) {
  std::string error_msg = "CommandHasError should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

void StateInit::PrintCurrentState(const PlanManagerStateMachine *state_machine,
                                  double t_now_seconds) const {
  std::string msg;
  msg += ("[INIT]: waiting for status message. ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ". t = ";
  msg +=
      std::to_string(state_machine->get_state_machine_up_time(t_now_seconds));
  spdlog::info(msg);
}

/* Idle State */
PlanManagerStateBase *StateIdle::instance_ = nullptr;
PlanManagerStateBase *StateIdle::Instance() {
  if (!instance_) {
    instance_ = new StateIdle();
  }
  return instance_;
}

void StateIdle::QueueNewPlan(PlanManagerStateMachine *state_machine,
                             std::unique_ptr<PlanBase> plan) {
  auto &plan_queue = state_machine->get_mutable_plans_queue();
  plan_queue.push(std::move(plan));
  ChangeState(state_machine, StateRunning::Instance());
}

void StateIdle::PrintCurrentState(const PlanManagerStateMachine *state_machine,
                                  double t_now_seconds) const {
  std::string msg;
  msg += ("[IDLE]: waiting for new plans. ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ". t = ";
  msg +=
      std::to_string(state_machine->get_state_machine_up_time(t_now_seconds));
  spdlog::info(msg);
}

/* Running State */
PlanManagerStateBase *StateRunning::instance_ = nullptr;
PlanManagerStateBase *StateRunning::Instance() {
  if (!instance_) {
    instance_ = new StateRunning();
  }
  return instance_;
}

const PlanBase *
StateRunning::GetCurrentPlan(PlanManagerStateMachine *state_machine,
                             double t_now, const State &state) const {
  auto &plans = state_machine->get_mutable_plans_queue();
  DRAKE_THROW_UNLESS(!plans.empty());
  const double *t_start = state_machine->get_current_plan_start_time();
  if (t_start == nullptr) {
    // current_plan_start_time_seconds_ is nullptr, meaning that there is no
    // active plan.
    state_machine->set_current_plan_start_time(t_now);
  } else {
    const double t_elapsed_seconds = state_machine->GetCurrentPlanUpTime(t_now);
    if (t_elapsed_seconds > plans.front()->duration() + 1.0) {
      // current plan has expired, switching to the next plan.
      // TODO: but we're moving towards not having a queue of plans in
      //  state_machine?
      state_machine->set_current_plan_start_time(t_now);
      plans.pop();
    }
  }
  if (plans.empty()) {
    state_machine->reset_current_plan_start_time();
    ChangeState(state_machine, StateIdle::Instance());
    return nullptr;
  }
  return plans.front().get();
}

double
StateRunning::GetCurrentPlanUpTime(const PlanManagerStateMachine *state_machine,
                                   double t_now) const {
  const double *t_start = state_machine->get_current_plan_start_time();
  DRAKE_THROW_UNLESS(t_start != nullptr);
  return t_now - *t_start;
}

void StateRunning::QueueNewPlan(PlanManagerStateMachine *state_machine,
                                std::unique_ptr<PlanBase> plan) {
  // TODO: what is the desired behavior when receiving a new plan while a plan
  // is still being executed? Discard or queue?
  // Discard is probably the better option? The client should be blocked
  // while a plan is executing. A vector of plans can be sent over LCM to
  // be queued when state is IDLE.
  spdlog::warn("[Running]: another plan is running. "
               "Received plan is discarded.");
}

void StateRunning::PrintCurrentState(
    const PlanManagerStateMachine *state_machine, double t_now_seconds) const {
  std::string msg;
  msg += ("[RUNNING]: executing a plan. ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ". t = ";
  msg +=
      std::to_string(state_machine->get_state_machine_up_time(t_now_seconds));
  spdlog::info(msg);
}

/* Error State */
PlanManagerStateBase *StateError::instance_ = nullptr;
PlanManagerStateBase *StateError::Instance() {
  if (!instance_) {
    instance_ = new StateError();
  }
  return instance_;
}

bool StateError::CommandHasError(const State &state, const Command &cmd,
                                 PlanManagerStateMachine *state_machine,
                                 const double q_threshold) {
  std::string error_msg = "CommandHasError should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

void StateError::PrintCurrentState(const PlanManagerStateMachine *state_machine,
                                   double t_now_seconds) const {
  std::string msg;
  msg += ("[ERROR]: ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ". t = ";
  msg +=
      std::to_string(state_machine->get_state_machine_up_time(t_now_seconds));
  spdlog::critical(msg);
}

void StateError::QueueNewPlan(PlanManagerStateMachine *state_machine,
                              std::unique_ptr<PlanBase> plan) {
  std::string msg("[ERROR]: received plan is discarded.");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ".";
  spdlog::critical(msg);
}
