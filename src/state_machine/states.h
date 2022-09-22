#pragma once
#include "state_machine/plan_manager_state_machine.h"

class StateInit : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();

  [[nodiscard]] bool has_received_status_msg() const override { return false; };

  void ReceiveNewStatusMsg(
      PlanManagerStateMachine *state_machine,
      const drake::lcmt_iiwa_status &msg_iiwa_status) const override;

  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::unique_ptr<PlanBase> plan) override;

  bool CommandHasError(const State &state, const Command &cmd,
                       PlanManagerStateMachine *state_machine,
                       const double q_threshold) override;

  void PrintCurrentState(const PlanManagerStateMachine *manager,
                         double t_now_seconds) const override;

  void AbortAllPlans(PlanManagerStateMachine *state_machine) override {
    throw std::runtime_error("AbortAllPlans should not be called in state " +
                             get_state_name());
  };

  [[nodiscard]] PlanManagerStateTypes get_state_type() const override {
    return PlanManagerStateTypes::kStateInit;
  };

private:
  StateInit() : PlanManagerStateBase("INIT"){};
  static PlanManagerStateBase *instance_;
};

class StateIdle : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();

  [[nodiscard]] bool has_received_status_msg() const override { return true; };

  void ReceiveNewStatusMsg(
      PlanManagerStateMachine *state_machine,
      const drake::lcmt_iiwa_status &msg_iiwa_status) const override{};

  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::unique_ptr<PlanBase> plan) override;

  void PrintCurrentState(const PlanManagerStateMachine *manager,
                         double t_now_seconds) const override;

  [[nodiscard]] PlanManagerStateTypes get_state_type() const override {
    return PlanManagerStateTypes::kStateIdle;
  };

private:
  StateIdle() : PlanManagerStateBase("IDLE"){};
  static PlanManagerStateBase *instance_;
};

class StateRunning : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();

  const PlanBase *
  GetCurrentPlan(PlanManagerStateMachine *state_machine, double t_now,
                 const drake::lcmt_iiwa_status &msg_iiwa_status) const override;

  double GetCurrentPlanUpTime(const PlanManagerStateMachine *state_machine,
                              double t_now) const override;

  [[nodiscard]] bool has_received_status_msg() const override { return true; };

  void ReceiveNewStatusMsg(
      PlanManagerStateMachine *state_machine,
      const drake::lcmt_iiwa_status &msg_iiwa_status) const override{};

  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::unique_ptr<PlanBase> plan) override;

  void PrintCurrentState(const PlanManagerStateMachine *state_machine,
                         double t_now_seconds) const override;

  [[nodiscard]] PlanManagerStateTypes get_state_type() const override {
    return PlanManagerStateTypes::kStateRunning;
  };

private:
  StateRunning() : PlanManagerStateBase("RUNNING"){};
  static PlanManagerStateBase *instance_;
};

class StateError : public PlanManagerStateBase {
public:
  static PlanManagerStateBase *Instance();

  void QueueNewPlan(PlanManagerStateMachine *state_machine,
                    std::unique_ptr<PlanBase> plan) override;

  bool CommandHasError(const State &state, const Command &cmd,
                       PlanManagerStateMachine *state_machine,
                       const double q_threshold) override;

  void PrintCurrentState(const PlanManagerStateMachine *state_machine,
                         double t_now_seconds) const override;

  [[nodiscard]] PlanManagerStateTypes get_state_type() const override {
    return PlanManagerStateTypes::kStateError;
  };

  void ReceiveNewStatusMsg(
      PlanManagerStateMachine *state_machine,
      const drake::lcmt_iiwa_status &msg_iiwa_status) const override{};

  void AbortAllPlans(PlanManagerStateMachine *state_machine) override {
    throw std::runtime_error("AbortAllPlans should not be called in state " +
                             get_state_name());
  };

private:
  StateError() : PlanManagerStateBase("ERROR"){};
  static PlanManagerStateBase *instance_;
};
