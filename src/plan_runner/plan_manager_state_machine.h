#pragma once
#include <memory>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "plan_base.h"

class PlanManagerStateBase;

class PlanManagerStateMachine {
public:
  PlanManagerStateMachine();
  inline const PlanBase *get_current_plan() const;
  double get_plan_time() const;
  inline bool has_received_status_msg() const;
  inline void receive_new_status_msg(PlanManagerStateMachine *manager) const;
  void QueueNewPlan(std::shared_ptr<PlanBase> plan) const;
  size_t num_plans() const { return plans_.size(); }

private:
  friend class PlanManagerStateBase;
  void ChangeState(PlanManagerStateBase *new_state);
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
  PlanManagerStateBase *state_{nullptr};
  std::vector<std::unique_ptr<PlanBase>> plans_;
};

class PlanManagerStateBase {
public:
  virtual const PlanBase *
  get_current_plan(const PlanManagerStateMachine *manager) const = 0;
  virtual double get_plan_time() const = 0;
  virtual bool has_received_status_msg() const = 0;
  virtual void receive_new_status_msg(PlanManagerStateMachine *manager) const = 0;
  virtual void QueueNewPlan(std::shared_ptr<PlanBase> plan) = 0;
  bool CheckCommandForError(const Command& cmd) const;
protected:
  static void ChangeState(PlanManagerStateMachine *manager,
                          PlanManagerStateBase *new_state) {
    manager->ChangeState(new_state);
  };
};

