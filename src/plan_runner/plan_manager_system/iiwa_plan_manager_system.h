#pragma once
#include "drake/systems/framework/leaf_system.h"
#include "drake_lcmtypes/drake/lcmt_iiwa_status.hpp"
#include "drake_lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_plan.hpp"

#include "../plans/iiwa_plan_factory.h"
#include "../state_machine/plan_manager_state_machine.h"

class IiwaPlanManagerSystem : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaPlanManagerSystem);

  explicit IiwaPlanManagerSystem(double control_period_seconds);

  const drake::systems::InputPort<double>& get_iiwa_status_input_port() const {
    return get_input_port(input_port_iiwa_status_idx_);
  }

  const drake::systems::InputPort<double>& get_robot_plan_input_port() const {
    return get_input_port(input_port_robot_plan_idx_);
  }

private:
  void CalcIiwaCommand(const drake::systems::Context<double>& context,
                       drake::lcmt_iiwa_command* cmd) const;

  void PrintCurrentState(const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* state) const;

  const double control_period_seconds_{};
  IiwaPlanFactory plan_factory_;
  std::unique_ptr<PlanManagerStateMachine> state_machine_;
  // signature of the last robot plan.
  // TODO: use the hash of the plan (concatenated q_knots as strings?) as the
  //  signature.
  mutable long last_robot_plan_utime_{-1};

  // input port and state indices.
  drake::systems::InputPortIndex input_port_iiwa_status_idx_;
  drake::systems::InputPortIndex input_port_robot_plan_idx_;

  // a constant reference to the MBP in plan_factory_.
  const drake::multibody::MultibodyPlant<double> &plant_;
};
