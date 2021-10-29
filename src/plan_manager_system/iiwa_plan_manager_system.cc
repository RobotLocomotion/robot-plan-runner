#include "drake/common/value.h"
#include <Eigen/Dense>

#include "plan_manager_system/iiwa_plan_manager_system.h"

using drake::lcmt_iiwa_command;
using drake::lcmt_iiwa_status;
using drake::lcmt_robot_plan;
using drake::systems::BasicVector;
using Eigen::VectorXd;

IiwaPlanManagerSystem::IiwaPlanManagerSystem(YAML::Node config)
    : config_(std::move(config)),
      control_period_seconds_(config["control_period"].as<double>()) {
  // Initialize state machine, plan factory.
  state_machine_ = std::make_unique<PlanManagerStateMachine>(0, config_);
  plan_factory_ = std::make_unique<IiwaPlanFactory>(config_);

  // Printing events.
  set_name("IiwaPlanManagerSystem");
  DeclarePeriodicPublishEvent(1.0, 0,
                              &IiwaPlanManagerSystem::PrintCurrentState);

  // Input ports.
  input_port_iiwa_status_idx_ =
      DeclareAbstractInputPort("lcmt_iiwa_status",
                               *drake::AbstractValue::Make(lcmt_iiwa_status()))
          .get_index();
  input_port_robot_plan_idx_ =
      DeclareAbstractInputPort("lcmt_robot_plan",
                               *drake::AbstractValue::Make(lcmt_robot_plan()))
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
  const auto &status_msg =
      get_iiwa_status_input_port().Eval<lcmt_iiwa_status>(context);
  const auto &msg_robot_plan =
      get_robot_plan_input_port().Eval<lcmt_robot_plan>(context);
  auto &cmd_msg = *cmd;
  // Handle new robot plan messages.
  if (msg_robot_plan.num_states > 0 and
      msg_robot_plan.utime != last_robot_plan_utime_) {
    // has new message.
    auto plan = plan_factory_->MakePlan(msg_robot_plan);
    state_machine_->QueueNewPlan(std::move(plan));
    last_robot_plan_utime_ = msg_robot_plan.utime;
  }

  // Handle new iiwa status messages.
  if (status_msg.num_joints == 0) {
    return;
  }

  const double t_now = context.get_time();
  auto plan = state_machine_->GetCurrentPlan(t_now, status_msg);
  state_machine_->ReceiveNewStatusMsg(status_msg);

  State s(Eigen::Map<const VectorXd>(status_msg.joint_position_measured.data(),
                                     status_msg.num_joints),
          Eigen::Map<const VectorXd>(status_msg.joint_velocity_estimated.data(),
                                     status_msg.num_joints),
          Eigen::Map<const VectorXd>(status_msg.joint_torque_external.data(),
                                     status_msg.num_joints));
  Command c;

  bool command_has_error{true};
  // Refer to iiwa_plan_manager.cc for more detailed documentation.
  plan = state_machine_->GetCurrentPlan(t_now, status_msg);
  state_machine_->ReceiveNewStatusMsg(status_msg);

  try {
    // Compute command.
    if (plan) {
      const double t_plan = state_machine_->GetCurrentPlanUpTime(t_now);
      plan->Step(s, control_period_seconds_, t_plan, &c);
    } else if (state_machine_->get_state_type() ==
               PlanManagerStateTypes::kStateIdle) {
      c.q_cmd = state_machine_->get_iiwa_position_command_idle();
      c.tau_cmd = Eigen::VectorXd::Zero(status_msg.num_joints);
    } else {
      // In state INIT or ERROR.
      // Send an empty iiwa command message with utime = -1.
      // Behavior with mock_station_simulation:
      //  throws with error message:
      //   IiwaCommandReceiver expected num_joints = 7, but received 0.
      // TODO: confirm the behavior of drake-iiwa-driver (real robot).
      //  What I think will happen:
      //  throws if cmd_msg.num_joints != 7, unless cmd_msg
      //  .utime == -1, in which case no command is sent to the robot.
      cmd_msg = lcmt_iiwa_command();
      cmd_msg.utime = -1;
      return;
    }

    // Check command for error.
    state_machine_->CheckCommandForError(s, c);

    // If no exception has been thrown thus far, there is no error in command.
    command_has_error = false;
  } catch (PlanException &e) {
    state_machine_->EnterErrorState();
    spdlog::critical(e.what());
  }

  // Check command for error.
  if (!command_has_error) {
    const int num_joints = status_msg.num_joints;
    cmd_msg.num_joints = num_joints;
    cmd_msg.num_torques = num_joints;
    cmd_msg.utime = status_msg.utime;
    cmd_msg.joint_position.resize(num_joints);
    cmd_msg.joint_torque.resize(num_joints);
    for (int i = 0; i < num_joints; i++) {
      cmd_msg.joint_position[i] = c.q_cmd[i];
      cmd_msg.joint_torque[i] = c.tau_cmd[i];
    }
  }
}

void IiwaPlanManagerSystem::PrintCurrentState(
    const drake::systems::Context<double> &context) const {
  state_machine_->PrintCurrentState(context.get_time());
}
