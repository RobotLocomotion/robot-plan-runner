#pragma once
#include "plan_manager_state_machine.h"

class StateError : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();

  const PlanBase *GetCurrentPlan(
      PlanManagerStateMachine *state_machine, double t_now,
      const drake::lcmt_iiwa_status &msg_iiwa_status) const override {
    return nullptr;
  };

  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::unique_ptr<PlanBase> plan) override;

  bool CommandHasError(const State &state, const Command &cmd,
                       PlanManagerStateMachine *state_machine) override;

  void PrintCurrentState(const PlanManagerStateMachine *state_machine,
                         double t_now_seconds) const override;

  [[nodiscard]] PlanManagerStateTypes get_state_type() const override {
    return PlanManagerStateTypes::kStateError;
  };

  void ReceiveNewStatusMsg(
      PlanManagerStateMachine *state_machine,
      const drake::lcmt_iiwa_status &msg_iiwa_status) const override{};

private:
  StateError() : PlanManagerStateBase("ERROR"){};
  static PlanManagerStateBase *instance_;
};
