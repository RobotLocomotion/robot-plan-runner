#pragma once

#include <yaml-cpp/yaml.h>

#include "drake/systems/framework/leaf_system.h"
#include "drake_lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "drake_lcmtypes/drake/lcmt_iiwa_status.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_plan.hpp"

#include "plans/iiwa_plan_factory.h"
#include "state_machine/plan_manager_state_machine.h"


class IiwaPlanManagerSystem : public drake::systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaPlanManagerSystem);
  IiwaPlanManagerSystem();

  const drake::systems::InputPort<double> &get_iiwa_status_input_port() const {
    return get_input_port(input_port_iiwa_status_idx_);
  }

  const drake::systems::OutputPort<double> &
  get_iiwa_command_output_port() const {
    return get_output_port(output_port_iiwa_command_idx_);
  }

private:
  void CalcIiwaCommand(const drake::systems::Context<double> &context,
                       drake::lcmt_iiwa_command *cmd) const;

  // input port and state indices.
  drake::systems::InputPortIndex input_port_iiwa_status_idx_;
  drake::systems::OutputPortIndex output_port_iiwa_command_idx_;

  // storage for the constant robot command.
  mutable std::unique_ptr<Eigen::VectorXd> joint_position_cmd_;
};
