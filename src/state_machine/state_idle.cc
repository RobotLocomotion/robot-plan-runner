#include "state_idle.h"
#include "state_running.h"

using std::cout;
using std::endl;

PlanManagerStateBase *StateIdle::instance_ = nullptr;
PlanManagerStateBase *StateIdle::Instance() {
  if (!instance_) {
    instance_ = new StateIdle();
  }
  return instance_;
}

const PlanBase *StateIdle::GetCurrentPlan(
    PlanManagerStateMachine *state_machine, double t_now,
    const drake::lcmt_iiwa_status &msg_iiwa_status) const {
  DRAKE_THROW_UNLESS(state_machine->num_plans() == 0);
  return nullptr;
}

void StateIdle::QueueNewPlan(PlanManagerStateMachine *state_machine,
                             std::unique_ptr<PlanBase> plan) {
  auto &plan_queue = state_machine->get_mutable_plans_queue();
  plan_queue.push(std::move(plan));
  ChangeState(state_machine, StateRunning::Instance());
}

void StateIdle::PrintCurrentState(const PlanManagerStateMachine *state_machine,
                                  double t_now_seconds) const {
  std::string msg("t = ");
  msg +=
      std::to_string(state_machine->get_state_machine_up_time(t_now_seconds));
  msg += (". [IDLE]: waiting for new plans. ");
  msg += "Number of plans: ";
  msg += std::to_string(state_machine->num_plans());
  msg += ".";
  cout << msg << endl;
}

void StateIdle::ReceiveNewStatusMsg(
    PlanManagerStateMachine *state_machine,
    const drake::lcmt_iiwa_status &msg_iiwa_status) const {
  if (!state_machine->is_iiwa_position_command_idle_set()) {
    // Should be here when transitioning from RUNNING
    // (1) by calling GetCurrentPlan, when the plan queue is empty;
    // (2) by calling AbortAllPlans, which empties the plan queue.
    state_machine->SetIiwaPositionCommandIdle(msg_iiwa_status);
  }
}
