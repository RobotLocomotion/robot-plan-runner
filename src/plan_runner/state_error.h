#pragma once
#include "plan_manager_state_machine.h"


class StateError : public PlanManagerStateBase {
 public:
  static PlanManagerStateBase *Instance();
  const PlanBase *
  get_current_plan(const PlanManagerStateMachine *state_machine) const override;
  double get_plan_time() const override { return -1; };
  bool has_received_status_msg() const override { return true; };
  void receive_new_status_msg(PlanManagerStateMachine *state_machine) const
  override;
  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::shared_ptr<PlanBase> plan) override;
  void PrintCurrentState(const PlanManagerStateMachine *state_machine) const override;
  PlanExecutionStatus get_plan_execution_status() const override {
    return PlanExecutionStatus::kError;
  };
 private:
  static PlanManagerStateBase *instance_;
};
