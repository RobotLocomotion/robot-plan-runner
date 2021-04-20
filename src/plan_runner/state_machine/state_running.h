#pragma once
#include "plan_manager_state_machine.h"

class StateRunning : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();

  const PlanBase *GetCurrentPlan(PlanManagerStateMachine *state_machine,
                                 double t_now) const override;

  double GetCurrentPlanUpTime(const PlanManagerStateMachine *state_machine,
                              double t_now) const override;

  [[nodiscard]] bool has_received_status_msg() const override { return true; };

  void receive_new_status_msg(
      PlanManagerStateMachine *state_machine) const override{};

  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::unique_ptr<PlanBase> plan) override;

  void PrintCurrentState(const PlanManagerStateMachine *state_machine,
                         double t_now_seconds) const override;

  [[nodiscard]] PlanManagerStateTypes get_state_type() const override {
    return PlanManagerStateTypes::kStateRunning;
  };

  bool CommandHasError(const State &state, const Command &cmd,
                       PlanManagerStateMachine *state_machine) override;

private:
  StateRunning() : PlanManagerStateBase("RUNNING"){};
  static PlanManagerStateBase *instance_;
};
