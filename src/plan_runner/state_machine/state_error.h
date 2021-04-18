#pragma once
#include "plan_manager_state_machine.h"

class StateError : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();

  const PlanBase *GetCurrentPlan(PlanManagerStateMachine *state_machine,
                                 double t_now) const override {
    return nullptr;
  };

  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::unique_ptr<PlanBase> plan) override;

  void PrintCurrentState(
      const PlanManagerStateMachine *state_machine) const override;

  [[nodiscard]] PlanManagerStateTypes get_state_type() const override {
    return PlanManagerStateTypes::kStateError;
  };

  void receive_new_status_msg(
      PlanManagerStateMachine *state_machine) const override{};

private:
  StateError() : PlanManagerStateBase("ERROR"){};
  static PlanManagerStateBase *instance_;
};
