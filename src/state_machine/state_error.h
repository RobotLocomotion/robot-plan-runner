#pragma once
#include "state_machine/plan_manager_state_machine.h"

class StateError : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();

  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::unique_ptr<PlanBase> plan) override;

  void CheckCommandForError(const State &state, const Command &cmd,
                            PlanManagerStateMachine *state_machine,
                            const double q_threshold) override;

  void EnterErrorState(PlanManagerStateMachine *state_machine) override;

  void PrintCurrentState(const PlanManagerStateMachine *state_machine,
                         double t_now_seconds) const override;

  [[nodiscard]] PlanManagerStateTypes get_state_type() const override {
    return PlanManagerStateTypes::kStateError;
  };

  void ReceiveNewStatusMsg(
      PlanManagerStateMachine *state_machine,
      const drake::lcmt_iiwa_status &msg_iiwa_status) const override{};

  void AbortAllPlans(PlanManagerStateMachine *state_machine) override {
    throw std::runtime_error(
        "AbortAllPlans should not be called in state " + get_state_name());
  };

private:
  StateError() : PlanManagerStateBase("ERROR"){};
  static PlanManagerStateBase *instance_;
};
