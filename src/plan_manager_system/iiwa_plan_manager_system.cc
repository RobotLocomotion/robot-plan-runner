#include "drake/common/value.h"
#include <Eigen/Dense>

#include "plan_manager_system/iiwa_plan_manager_system.h"

using drake::lcmt_iiwa_command;
using drake::lcmt_iiwa_status;
using drake::lcmt_robot_plan;
using drake::systems::BasicVector;
using Eigen::VectorXd;

IiwaPlanManagerSystem::IiwaPlanManagerSystem() {
  set_name("IiwaPlanManagerSystem");

  // Input ports.
  input_port_iiwa_status_idx_ =
      DeclareAbstractInputPort("lcmt_iiwa_status",
                               *drake::AbstractValue::Make(lcmt_iiwa_status()))
          .get_index();

  // Output port.
  // The rate at which this output port is evaluated is driven by the
  //  downstream system. In lcm interface, the update rate is defined in the
  //  downstream LcmPublisherSystem. In simulation without lcm, an
  //  abstract-valued ZeroOrderHold is needed to enforce the update rate.
  output_port_iiwa_command_idx_ =
      DeclareAbstractOutputPort("lcmt_iiwa_command",
                                &IiwaPlanManagerSystem::CalcIiwaCommand)
          .get_index();
}

void IiwaPlanManagerSystem::CalcIiwaCommand(
    const drake::systems::Context<double> &context,
    drake::lcmt_iiwa_command *cmd) const {
  const auto &msg_iiwa_status =
      get_iiwa_status_input_port().Eval<lcmt_iiwa_status>(context);
  auto &msg_iiwa_command = *cmd;
  const int num_joints = msg_iiwa_status.num_joints;

  // Handle new iiwa status messages.
  if (num_joints == 0) {
    return;
  }

  // Create constant joint command message if it doesn't exist yet.
  if (joint_position_cmd_ == nullptr) {
    joint_position_cmd_ =
        std::make_unique<VectorXd>(num_joints);
    for (size_t i = 0; i <num_joints; i++) {
      (*joint_position_cmd_)[i] = msg_iiwa_status.joint_position_measured[i];
    }
  }

  // Send cmd message with cmd.utime == status.utime.
  msg_iiwa_command.num_joints = num_joints;
  msg_iiwa_command.num_torques = num_joints;
  msg_iiwa_command.utime = msg_iiwa_status.utime;
  msg_iiwa_command.joint_position.resize(num_joints);
  msg_iiwa_command.joint_torque.resize(num_joints);
  for (int i = 0; i < num_joints; i++) {
    msg_iiwa_command.joint_position[i] = (*joint_position_cmd_)[i];
    msg_iiwa_command.joint_torque[i] = 0;
  }
}

