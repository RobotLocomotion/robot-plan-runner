#pragma once
#include "plan_manager_state_machine.h"

class StateError : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();
  const PlanBase *GetCurrentPlan(
      PlanManagerStateMachine *state_machine) const override {
    return nullptr;
  };
  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::shared_ptr<PlanBase> plan) override;
  void PrintCurrentState(
      const PlanManagerStateMachine *state_machine) const override;
  [[nodiscard]] PlanExecutionStatus get_plan_execution_status() const override {
    return PlanExecutionStatus::kError;
  };

private:
  StateError() : PlanManagerStateBase("ERROR"){};
  static PlanManagerStateBase *instance_;
};
