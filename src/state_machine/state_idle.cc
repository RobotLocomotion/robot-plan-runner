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

