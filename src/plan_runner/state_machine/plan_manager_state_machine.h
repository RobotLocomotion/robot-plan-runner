#pragma once
#include <chrono>
#include <memory>
#include <queue>

#include "drake/multibody/plant/multibody_plant.h"
#include "../plans/plan_base.h"

// TODO: this is not used right now.
enum PlanManagerStateTypes {
  kStateInit,
  kStateIdle,
  kStateRunning,
  kStateError
};
typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePoint;
class PlanManagerStateBase;

class PlanManagerStateMachine {
public:
  PlanManagerStateMachine();
  // State-dependent methods.
  [[nodiscard]] const PlanBase *GetCurrentPlan(const TimePoint &t_now);

  // Returns in seconds how long the current plan has been active.
  [[nodiscard]] double GetCurrentPlanUpTime(const TimePoint &t_now) const;

  // Print information about the currently active state.
  void PrintCurrentState() const;

  // Tries to add a plan to the queue of plans to be executed. Note that the
  // maximum size of this queue is 1 in the current implementation.
  void QueueNewPlan(std::unique_ptr<PlanBase> plan);

  // Checks a command computed by the plan.Step() function for errors.
  // Currently checks for:
  // 1. Nans
  // 2. If cmd.q_cmd and state.q is too far away with a hard-coded threshold.
  bool CommandHasError(const State &state, const Command &cmd);

  // Returns true if an IIWA_STATUS message has been received.
  [[nodiscard]] bool has_received_status_msg() const;

  // Called when a new IIWA_STATUS message is received. If the current state is
  //  INIT, the state is changed to IDLE. If the current state is IDLE,
  //  RUNNING or ERROR, this function does nothing.
  void receive_new_status_msg();

  [[nodiscard]] PlanManagerStateTypes get_state_type() const;

  // Other methods.
  [[nodiscard]] size_t num_plans() const { return plans_.size(); }

  std::queue<std::unique_ptr<PlanBase>> &get_mutable_plans_queue() {
    return plans_;
  };

  [[nodiscard]] const std::queue<std::unique_ptr<PlanBase>> &
  get_plans_queue() const {
    return plans_;
  };

  // TODO: "time" methods should probably be private. Access by states can be
  //  enabled by forwarding in PlanManagerStateBase.
  void set_current_plan_start_time(const TimePoint &t);

  void reset_current_plan_start_time() { current_plan_start_time_.reset(); };

  [[nodiscard]] const TimePoint *get_current_plan_start_time() const {
    return current_plan_start_time_.get();
  };

private:
  friend class PlanManagerStateBase;
  inline void ChangeState(PlanManagerStateBase *new_state);
  PlanManagerStateBase *state_{nullptr};
  std::queue<std::unique_ptr<PlanBase>> plans_;
  std::unique_ptr<TimePoint> current_plan_start_time_{nullptr};
};

class PlanManagerStateBase {
public:
  // Virtual functions.
  [[nodiscard]] virtual double
  GetCurrentPlanUpTime(const PlanManagerStateMachine *state_machine,
                       const TimePoint &t_now) const;

  [[nodiscard]] virtual bool has_received_status_msg() const;

  virtual void
  receive_new_status_msg(PlanManagerStateMachine *state_machine) const;

  virtual void QueueNewPlan(PlanManagerStateMachine *state_machine,
                            std::unique_ptr<PlanBase> plan);

  virtual bool CommandHasError(const State &state, const Command &cmd,
                               PlanManagerStateMachine *state_machine);
  // Pure virtual functions.
  [[nodiscard]] virtual PlanManagerStateTypes get_state_type() const = 0;

  virtual void
  PrintCurrentState(const PlanManagerStateMachine *state_machine) const = 0;

  virtual const PlanBase *GetCurrentPlan(PlanManagerStateMachine *state_machine,
                                         const TimePoint &t_now) const = 0;

  // Other functions.
  [[nodiscard]] const std::string &get_state_name() const {
    return state_name_;
  };

protected:
  explicit PlanManagerStateBase(std::string state_name)
      : state_name_(std::move(state_name)){};

  static void ChangeState(PlanManagerStateMachine *state_machine,
                          PlanManagerStateBase *new_state) {
    state_machine->ChangeState(new_state);
  };

private:
  const std::string state_name_;
};

inline bool PlanManagerStateMachine::has_received_status_msg() const {
  return state_->has_received_status_msg();
}

inline void PlanManagerStateMachine::receive_new_status_msg() {
  state_->receive_new_status_msg(this);
}

inline PlanManagerStateTypes PlanManagerStateMachine::get_state_type() const {
  return state_->get_state_type();
}

inline void
PlanManagerStateMachine::QueueNewPlan(std::unique_ptr<PlanBase> plan) {
  state_->QueueNewPlan(this, std::move(plan));
}

inline void
PlanManagerStateMachine::ChangeState(PlanManagerStateBase *new_state) {
  state_ = new_state;
}

inline double
PlanManagerStateMachine::GetCurrentPlanUpTime(const TimePoint &t_now) const {
  return state_->GetCurrentPlanUpTime(this, t_now);
}

inline const PlanBase *
PlanManagerStateMachine::GetCurrentPlan(const TimePoint &t_now) {
  return state_->GetCurrentPlan(this, t_now);
}

inline void PlanManagerStateMachine::PrintCurrentState() const {
  state_->PrintCurrentState(this);
}

inline void
PlanManagerStateMachine::set_current_plan_start_time(const TimePoint &t) {
  current_plan_start_time_ = std::make_unique<TimePoint>(t);
}

inline bool PlanManagerStateMachine::CommandHasError(const State &state,
                                                     const Command &cmd) {
  return state_->CommandHasError(state, cmd, this);
}
