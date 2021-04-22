#include "plan_manager_state_machine.h"
#include "state_init.h"
#include "state_error.h"

using std::string;

PlanManagerStateMachine::PlanManagerStateMachine(
    double state_machine_start_time_seconds)
    : state_machine_start_time_seconds_(state_machine_start_time_seconds) {
  // Initialize to state INIT.
  state_ = StateInit::Instance();
}

void PlanManagerStateMachine::SetIiwaPositionCommandIdle(
    const drake::lcmt_iiwa_status &msg_iiwa_status) {
  iiwa_position_command_idle_ = Eigen::Map<const Eigen::VectorXd>(
      msg_iiwa_status.joint_position_measured.data(),
      msg_iiwa_status.num_joints);
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

bool PlanManagerStateBase::CommandHasError(const State & state,
                                           const Command & cmd,
                                           PlanManagerStateMachine *state_machine,
                                           const double q_threshold) {
  bool is_nan =
      cmd.q_cmd.array().isNaN().sum() or cmd.tau_cmd.array().isNaN().sum();

  bool is_too_far_away = (state.q - cmd.q_cmd).norm() > q_threshold;

  bool is_error = is_nan or is_too_far_away;
  if (is_error) {
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

