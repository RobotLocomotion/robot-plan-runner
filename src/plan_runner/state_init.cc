#include "state_init.h"
#include "state_idle.h"

PlanManagerStateBase* StateInit::instance_ = nullptr;
PlanManagerStateBase* StateInit::Instance() {
  if (!instance_) {
    instance_ = new StateInit();
  }
  return instance_;
}

const PlanBase* StateInit::get_current_plan(const PlanManagerStateMachine*
manager) const {
  DRAKE_THROW_UNLESS(manager->num_plans() == 0);
  return nullptr;
}

void StateInit::receive_new_status_msg(PlanManagerStateMachine *manager) const {
  ChangeState(manager, StateIdle::Instance());
}

void StateInit::QueueNewPlan(std::shared_ptr<PlanBase> plan) {
  std::cout << "[RobotPlanRunner]: no robot status message received yet." <<
            std::endl;
}
