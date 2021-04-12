#pragma once
#include "plan_manager_state_machine.h"

class StateRunning : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();
  const PlanBase *
  GetCurrentPlan(PlanManagerStateMachine *state_machine) const override;
  double get_current_plan_up_time(
      const PlanManagerStateMachine *state_machine) const override;
  [[nodiscard]] bool has_received_status_msg() const override { return true; };
  void receive_new_status_msg(
      PlanManagerStateMachine *state_machine) const override{};
  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::shared_ptr<PlanBase> plan) override;
  void PrintCurrentState(
      const PlanManagerStateMachine *state_machine) const override;
  [[nodiscard]] PlanExecutionStatus get_plan_execution_status() const override;
  bool CommandHasError(const State &state, const Command &cmd,
                       PlanManagerStateMachine *state_machine) override;

private:
  StateRunning() : PlanManagerStateBase("RUNNING"){};
  static PlanManagerStateBase *instance_;
};
