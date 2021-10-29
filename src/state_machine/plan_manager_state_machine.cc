#include "state_machine/plan_manager_state_machine.h"
#include "state_machine/state_error.h"
#include "state_machine/state_idle.h"
#include "state_machine/state_init.h"

using std::cout;
using std::endl;
using std::string;

PlanManagerStateMachine::PlanManagerStateMachine(
    double state_machine_start_time_seconds, const YAML::Node &config)
    : state_machine_start_time_seconds_(state_machine_start_time_seconds),
      config_(config) {
  // Initialize to state INIT.
  state_ = StateInit::Instance();
}

void PlanManagerStateMachine::SetIiwaPositionCommandIdle(
    const Eigen::Ref<const Eigen::VectorXd> &q_cmd) {
  if (iiwa_position_command_idle_) {
    DRAKE_THROW_UNLESS(q_cmd.size() == iiwa_position_command_idle_->size());
    *iiwa_position_command_idle_ = q_cmd;
  } else {
    iiwa_position_command_idle_ = std::make_unique<Eigen::VectorXd>(q_cmd);
  }
}

const PlanBase *PlanManagerStateBase::GetCurrentPlan(
    PlanManagerStateMachine *state_machine, double t_now,
    const drake::lcmt_iiwa_status &msg_iiwa_status) const {
  DRAKE_THROW_UNLESS(state_machine->num_plans() == 0);
  return nullptr;
}

double PlanManagerStateBase::GetCurrentPlanUpTime(
    const PlanManagerStateMachine *state_machine, double t_now) const {
  string error_msg = "GetCurrentPlanUpTime should not be called in state ";
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

bool PlanManagerStateBase::CommandHasError(
    const State &state, const Command &cmd,
    PlanManagerStateMachine *state_machine, const double q_threshold) {
  bool is_nan =
      cmd.q_cmd.array().isNaN().sum() or cmd.tau_cmd.array().isNaN().sum();

  bool is_too_far_away = (state.q - cmd.q_cmd).norm() > q_threshold;
  bool is_error = is_nan or is_too_far_away;
  if (is_error) {
    while (!state_machine->plans_.empty()) {
      // Delete all plans.
      state_machine->plans_.pop();
    }
    ChangeState(state_machine, StateError::Instance());
  }
  return is_error;
}

bool PlanManagerStateBase::has_received_status_msg() const {
  string error_msg = "has_received_status_msg should not be called in state ";
  error_msg += get_state_name();
  error_msg += ".";
  throw std::runtime_error(error_msg);
}

void PlanManagerStateBase::AbortAllPlans(
    PlanManagerStateMachine *state_machine) {
  ChangeState(state_machine, StateIdle::Instance());
  auto &plans_queue = state_machine->get_mutable_plans_queue();
  while (!plans_queue.empty()) {
    plans_queue.pop();
  }
  state_machine->reset_current_plan_start_time();
  spdlog::flush_on(spdlog::level::info);
  spdlog::info("All plans have been aborted.");
}
