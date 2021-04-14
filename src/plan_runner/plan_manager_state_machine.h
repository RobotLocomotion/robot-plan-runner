#pragma once
#include <chrono>
#include <memory>
#include <queue>

#include "drake/multibody/plant/multibody_plant.h"
#include "plans/plan_base.h"

enum PlanExecutionStatus { kFinished, kError, kNoActivePlan };

class PlanManagerStateBase;

class PlanManagerStateMachine {
public:
  PlanManagerStateMachine();
  // State-dependent methods.
  [[nodiscard]] inline const PlanBase *GetCurrentPlan();
  [[nodiscard]] inline bool has_received_status_msg() const;
  inline void receive_new_status_msg();
  [[nodiscard]] inline PlanExecutionStatus get_plan_execution_status() const;
  [[nodiscard]] inline double get_current_plan_up_time() const;
  inline void PrintCurrentState() const;
  void QueueNewPlan(std::shared_ptr<PlanBase> plan);
  bool CommandHasError(const State &state, const Command &cmd);

  // Other methods.
  [[nodiscard]] size_t num_plans() const { return plans_.size(); }
  std::queue<std::shared_ptr<PlanBase>> &get_mutable_plans_queue() {
    return plans_;
  };
  // TODO: replace shared_ptr with unique_ptr (or something else?).
  [[nodiscard]] const std::queue<std::shared_ptr<PlanBase>> &
  get_plans_queue() const {
    return plans_;
  };

  // TODO: "time" methods should probably be private. Access by states can be
  //  enabled by forwarding in PlanManagerStateBase.
  void set_current_plan_start_time(
      const std::chrono::time_point<std::chrono::high_resolution_clock> &t);
  void reset_current_plan_start_time() { current_plan_start_time_.reset(); };
  [[nodiscard]] double get_current_plan_up_time(
      const std::chrono::time_point<std::chrono::high_resolution_clock> &t_now)
      const;

private:
  friend class PlanManagerStateBase;
  inline void ChangeState(PlanManagerStateBase *new_state);
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
  PlanManagerStateBase *state_{nullptr};
  std::queue<std::shared_ptr<PlanBase>> plans_;
  std::unique_ptr<std::chrono::time_point<std::chrono::high_resolution_clock>>
      current_plan_start_time_{nullptr};
};

class PlanManagerStateBase {
public:
  // Virtual functions.
  [[nodiscard]] virtual double get_current_plan_up_time(
      const PlanManagerStateMachine *state_machine) const;
  [[nodiscard]] virtual bool has_received_status_msg() const;
  virtual void
  receive_new_status_msg(PlanManagerStateMachine *state_machine) const;
  virtual void QueueNewPlan(PlanManagerStateMachine *state_machine,
                            std::shared_ptr<PlanBase> plan);
  virtual bool CommandHasError(const State &state, const Command &cmd,
                               PlanManagerStateMachine *state_machine);
  // Pure virtual functions.
  [[nodiscard]] virtual PlanExecutionStatus
  get_plan_execution_status() const = 0;
  virtual void
  PrintCurrentState(const PlanManagerStateMachine *state_machine) const = 0;
  virtual const PlanBase *
  GetCurrentPlan(PlanManagerStateMachine *state_machine) const = 0;

  // Other functions.
  [[nodiscard]] const std::string &get_state_name() const {
    return state_name_;
  };

protected:
  PlanManagerStateBase(const std::string &state_name)
      : state_name_(state_name){};
  static void ChangeState(PlanManagerStateMachine *state_machine,
                          PlanManagerStateBase *new_state) {
    state_machine->ChangeState(new_state);
  };

private:
  const std::string state_name_;
};

inline const PlanBase *PlanManagerStateMachine::GetCurrentPlan() {
  return state_->GetCurrentPlan(this);
}

inline bool PlanManagerStateMachine::has_received_status_msg() const {
  return state_->has_received_status_msg();
}

inline void PlanManagerStateMachine::receive_new_status_msg() {
  state_->receive_new_status_msg(this);
}

inline PlanExecutionStatus
PlanManagerStateMachine::get_plan_execution_status() const {
  return state_->get_plan_execution_status();
}

inline double PlanManagerStateMachine::get_current_plan_up_time() const {
  return state_->get_current_plan_up_time(this);
}

inline void
PlanManagerStateMachine::QueueNewPlan(std::shared_ptr<PlanBase> plan) {
  state_->QueueNewPlan(this, plan);
}

inline void
PlanManagerStateMachine::ChangeState(PlanManagerStateBase *new_state) {
  state_ = new_state;
}

inline void PlanManagerStateMachine::PrintCurrentState() const {
  state_->PrintCurrentState(this);
};
