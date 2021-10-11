#pragma once
#include <chrono>
#include <memory>
#include <queue>
#include <sstream>
#include <yaml-cpp/yaml.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake_lcmtypes/drake/lcmt_iiwa_status.hpp"

#include "plans/plan_base.h"

// TODO: this is not used right now.
enum PlanManagerStateTypes {
  kStateInit,
  kStateIdle,
  kStateRunning,
  kStateError
};
using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
using DoubleSeconds = std::chrono::duration<double, std::ratio<1, 1>>;
class PlanManagerStateBase;

class PlanManagerStateMachine {
public:
  explicit PlanManagerStateMachine(double state_machine_start_time_seconds,
                                   const YAML::Node &config);
  // State-dependent methods.
  // TODO: separate the logic that schedules plans into another function.
  [[nodiscard]] const PlanBase *
  GetCurrentPlan(const TimePoint &t_now,
                 const drake::lcmt_iiwa_status &msg_iiwa_status) {
    double t_now_double =
        std::chrono::duration_cast<DoubleSeconds>(t_now.time_since_epoch())
            .count();
    return state_->GetCurrentPlan(this, t_now_double, msg_iiwa_status);
  }

  [[nodiscard]] const PlanBase *
  GetCurrentPlan(double t_now_seconds,
                 const drake::lcmt_iiwa_status &msg_iiwa_status) {
    return state_->GetCurrentPlan(this, t_now_seconds, msg_iiwa_status);
  }

  // Returns in seconds how long the current plan has been active.
  [[nodiscard]] double GetCurrentPlanUpTime(const TimePoint &t_now) const {
    double t_now_double =
        std::chrono::duration_cast<DoubleSeconds>(t_now.time_since_epoch())
            .count();
    return state_->GetCurrentPlanUpTime(this, t_now_double);
  }

  [[nodiscard]] double GetCurrentPlanUpTime(double t_now_seconds) const {
    return state_->GetCurrentPlanUpTime(this, t_now);
  }

  // Print information about the currently active state.
  void PrintCurrentState(double t_now_seconds) const {
    state_->PrintCurrentState(this, t_now_seconds);
  }

  // Tries to add a plan to the queue of plans to be executed. Note that the
  // maximum size of this queue is 1 in the current implementation.
  void QueueNewPlan(std::unique_ptr<PlanBase> plan) {
    state_->QueueNewPlan(this, std::move(plan));
  }

  // Checks a command computed by the plan.Step() function for errors.
  // Currently checks for:
  // 1. Nans
  // 2. If cmd.q_cmd and state.q is too far away with a hard-coded threshold.
  //    This threshold is decided by a parameter in the config file.
  bool CommandHasError(const State &state, const Command &cmd) {
    return state_->CommandHasError(state, cmd, this,
                                   config_["q_threshold"].as<double>());
  }

  // Empties the plans_ queue and sets the state to IDLE.
  void AbortAllPlans() { state_->AbortAllPlans(this); }

  // Returns true if an IIWA_STATUS message has been received.
  [[nodiscard]] bool has_received_status_msg() const {
    return state_->has_received_status_msg();
  }

  // Called when a new IIWA_STATUS message is received. If the current state is
  //  INIT, the state is changed to IDLE. If the current state is IDLE,
  //  RUNNING or ERROR, this function does nothing.
  void ReceiveNewStatusMsg(const drake::lcmt_iiwa_status &msg_iiwa_status) {
    state_->ReceiveNewStatusMsg(this, msg_iiwa_status);
  }

  [[nodiscard]] PlanManagerStateTypes get_state_type() const {
    return state_->get_state_type();
  }

  // Other methods.
  [[nodiscard]] size_t num_plans() const { return plans_.size(); }

  std::queue<std::unique_ptr<PlanBase>> &get_mutable_plans_queue() {
    return plans_;
  };

  [[nodiscard]] const std::queue<std::unique_ptr<PlanBase>> &
  get_plans_queue() const {
    return plans_;
  };

  // Stores lcmt_iiwa_status.joint_position_measured in
  //  iiwa_position_command_idle_.
  void
  SetIiwaPositionCommandIdle(const Eigen::Ref<const Eigen::VectorXd> &q_cmd);

  [[nodiscard]] const Eigen::VectorXd &get_iiwa_position_command_idle() const {
    return *iiwa_position_command_idle_;
  };

  void reset_iiwa_position_command_idle() {
    iiwa_position_command_idle_.reset();
  }

  [[nodiscard]] bool is_iiwa_position_command_idle_set() const {
    return iiwa_position_command_idle_ != nullptr;
  }

  [[nodiscard]] double get_state_machine_up_time(double t_now_seconds) const {
    return t_now_seconds - state_machine_start_time_seconds_;
  }

  // TODO: "time" methods should probably be private. Access by states can be
  //  enabled by forwarding in PlanManagerStateBase.
  void set_current_plan_start_time(double t_now_seconds) {
    current_plan_start_time_seconds_ = std::make_unique<double>(t_now_seconds);
  }

  void reset_current_plan_start_time() {
    current_plan_start_time_seconds_.reset();
  };

  [[nodiscard]] const double *get_current_plan_start_time() const {
    return current_plan_start_time_seconds_.get();
  };

private:
  friend class PlanManagerStateBase;

  inline void ChangeState(PlanManagerStateBase *new_state) {
    state_ = new_state;
  }

  PlanManagerStateBase *state_{nullptr};
  std::queue<std::unique_ptr<PlanBase>> plans_;

  const YAML::Node &config_;

  // In PlanManagerStateMachine, this stores TimePoint::time_since_epoch() in
  // seconds as a double.
  // In Drake systems, this stores context.get_time().
  std::unique_ptr<double> current_plan_start_time_seconds_{nullptr};

  const double state_machine_start_time_seconds_;

  // The iiwa command to send in state IDLE.
  std::unique_ptr<Eigen::VectorXd> iiwa_position_command_idle_{nullptr};
};

class PlanManagerStateBase {
public:
  // Virtual functions.
  [[nodiscard]] virtual double
  GetCurrentPlanUpTime(const PlanManagerStateMachine *state_machine,
                       double t_now) const;

  [[nodiscard]] virtual bool has_received_status_msg() const;

  virtual void QueueNewPlan(PlanManagerStateMachine *state_machine,
                            std::unique_ptr<PlanBase> plan);

  virtual bool CommandHasError(const State &state, const Command &cmd,
                               PlanManagerStateMachine *state_machine,
                               const double q_threshold);

  virtual void AbortAllPlans(PlanManagerStateMachine *state_machine);

  virtual const PlanBase *
  GetCurrentPlan(PlanManagerStateMachine *state_machine, double t_now_seconds,
                 const drake::lcmt_iiwa_status &msg_iiwa_status) const;

  // Pure virtual functions.
  [[nodiscard]] virtual PlanManagerStateTypes get_state_type() const = 0;

  virtual void
  ReceiveNewStatusMsg(PlanManagerStateMachine *state_machine,
                      const drake::lcmt_iiwa_status &msg_iiwa_status) const = 0;
  virtual void PrintCurrentState(const PlanManagerStateMachine *state_machine,
                                 double t_now_seconds) const = 0;

  // Other functions.
  [[nodiscard]] const std::string &get_state_name() const {
    return state_name_;
  };

protected:
  explicit PlanManagerStateBase(std::string state_name)
      : state_name_(std::move(state_name)){};

  static void ChangeState(PlanManagerStateMachine *state_machine,
                          PlanManagerStateBase *new_state) {
    std::stringstream ss;
    ss << "[" << state_machine->state_->get_state_name() << "]"
       << "---->"
       << "[" << new_state->get_state_name() << "]";
    spdlog::info(ss.str());
    state_machine->ChangeState(new_state);
  };

private:
  const std::string state_name_;
};
