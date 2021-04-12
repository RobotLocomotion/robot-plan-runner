#pragma once
#include <memory>
#include <queue>

#include "drake/multibody/plant/multibody_plant.h"
#include "plan_base.h"

enum PlanExecutionStatus { kFinished, kError, kNoActivePlan };

class PlanManagerStateBase;

class PlanManagerStateMachine {
public:
  PlanManagerStateMachine();
  // State-dependent methods.
  inline const PlanBase *get_current_plan() const;
  inline bool has_received_status_msg() const;
  inline void receive_new_status_msg();
  inline PlanExecutionStatus get_plan_execution_status() const;
  inline double get_plan_time() const;
  void PrintCurrentState() const;
  void QueueNewPlan(std::shared_ptr<PlanBase> plan);
  bool CheckCommandForError(const Command &cmd, const State &state) const;

  // Other methods.
  size_t num_plans() const { return plans_.size(); }
  std::queue<std::shared_ptr<PlanBase>> &get_mutable_plans_queue() {
    return plans_;
  };
  // TODO: replace shared_ptr with unique_ptr.
  const std::queue<std::shared_ptr<PlanBase>> &get_plans_queue() const {
    return plans_;
  }

private:
  friend class PlanManagerStateBase;
  inline void ChangeState(PlanManagerStateBase *new_state);
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
  PlanManagerStateBase *state_{nullptr};
  std::queue<std::shared_ptr<PlanBase>> plans_;
};

class PlanManagerStateBase {
public:
  virtual const PlanBase *
  get_current_plan(const PlanManagerStateMachine *state_machine) const = 0;
  virtual double get_plan_time() const = 0;
  virtual bool has_received_status_msg() const = 0;
  virtual void
  receive_new_status_msg(PlanManagerStateMachine *state_machine) const = 0;
  virtual PlanExecutionStatus get_plan_execution_status() const = 0;
  virtual void QueueNewPlan(PlanManagerStateMachine *state_machine,
                            std::shared_ptr<PlanBase> plan) = 0;
  virtual void
  PrintCurrentState(const PlanManagerStateMachine *state_machine) const = 0;
  virtual bool CheckCommandForError(const Command &cmd,
                                    const State &state) const = 0;

protected:
  static void ChangeState(PlanManagerStateMachine *state_machine,
                          PlanManagerStateBase *new_state) {
    state_machine->ChangeState(new_state);
  };
};

inline const PlanBase *PlanManagerStateMachine::get_current_plan() const {
  return state_->get_current_plan(this);
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

inline double PlanManagerStateMachine::get_plan_time() const {
  return state_->get_plan_time();
}

inline void
PlanManagerStateMachine::QueueNewPlan(std::shared_ptr<PlanBase> plan) {
  state_->QueueNewPlan(this, plan);
}

inline void
PlanManagerStateMachine::ChangeState(PlanManagerStateBase *new_state) {
  state_ = new_state;
}

bool PlanManagerStateMachine::CheckCommandForError(const Command &cmd,
                                                   const State &state) const {
  state_->CheckCommandForError(cmd, state);
}
