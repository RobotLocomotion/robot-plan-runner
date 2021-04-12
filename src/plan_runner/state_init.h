#pragma once
#include "plan_manager_state_machine.h"

class StateInit : public PlanManagerStateBase {
 public:
  static PlanManagerStateBase *Instance();
  const PlanBase *
  GetCurrentPlan(PlanManagerStateMachine *state_machine) const override;
  [[nodiscard]] bool has_received_status_msg() const override { return false; };
  void receive_new_status_msg(PlanManagerStateMachine *state_machine) const override;
  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::shared_ptr<PlanBase> plan) override;
  void PrintCurrentState(const PlanManagerStateMachine *manager) const override;
  [[nodiscard]] PlanExecutionStatus get_plan_execution_status() const override {
    return PlanExecutionStatus::kNoActivePlan;
  };
 private:
  StateInit() : PlanManagerStateBase("INIT") {};
  static PlanManagerStateBase *instance_;
};
