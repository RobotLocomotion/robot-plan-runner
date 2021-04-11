#pragma once
#include "plan_manager_state_machine.h"

class StateIdle : public PlanManagerStateBase {
 public:
  static PlanManagerStateBase *Instance();
  const PlanBase *
  get_current_plan(const PlanManagerStateMachine *manager) const override;
  double get_plan_time() const override { return -1; };
  bool has_received_status_msg() const override { return false; };
  void receive_new_status_msg(PlanManagerStateMachine *manager) const override;
  void QueueNewPlan(std::shared_ptr<PlanBase> plan) override;

 private:
  static PlanManagerStateBase *instance_;
};