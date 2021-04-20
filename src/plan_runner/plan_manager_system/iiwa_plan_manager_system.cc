#include "drake/common/value.h"
#include <Eigen/Dense>

#include "iiwa_plan_manager_system.h"

using drake::lcmt_iiwa_command;
using drake::lcmt_iiwa_status;
using drake::lcmt_robot_plan;
using drake::systems::BasicVector;
using Eigen::VectorXd;

IiwaPlanManagerSystem::IiwaPlanManagerSystem(double control_period_seconds)
    : control_period_seconds_(control_period_seconds),
      plant_(plan_factory_.get_plant()) {
  // Initialize state machine.
  state_machine_ = std::make_unique<PlanManagerStateMachine>(0);

  set_name("IiwaPlanManagerSystem");
  // Abstract state and update events.
  // At least one state is needed for declaring an discrete update event, but
  //  only printing is done in that event, therefore this state is not updated.
  DeclareDiscreteState(1);
  DeclarePeriodicDiscreteUpdateEvent(
      1.0, 0, &IiwaPlanManagerSystem::PrintCurrentState);

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
                            &IiwaPlanManagerSystem::CalcIiwaCommand).get_index();
}

void IiwaPlanManagerSystem::CalcIiwaCommand(
    const drake::systems::Context<double> &context,
    drake::lcmt_iiwa_command* cmd) const {
  const auto &msg_iiwa_status =
      get_iiwa_status_input_port().Eval<lcmt_iiwa_status>(context);
  const auto &msg_robot_plan =
      get_robot_plan_input_port().Eval<lcmt_robot_plan>(context);
  auto &msg_iiwa_command = *cmd;
  // Handle new robot plan messages.
  if (msg_robot_plan.num_states > 0 and
      msg_robot_plan.utime != last_robot_plan_utime_) {
    // has new message.
    auto plan = plan_factory_.MakePlan(msg_robot_plan);
    state_machine_->QueueNewPlan(std::move(plan));
    last_robot_plan_utime_ = msg_robot_plan.utime;
  }

  // Handle new iiwa status messages.
  if (msg_iiwa_status.num_joints == 0) {
    return;
  } else {
    state_machine_->ReceiveNewStatusMsg(msg_iiwa_status);
  }

  // Compute iiwa_command.
  const double t_now = context.get_time();
  auto plan = state_machine_->GetCurrentPlan(t_now, msg_iiwa_status);

  State s(
      Eigen::Map<const VectorXd>(msg_iiwa_status.joint_position_measured.data(),
                                 msg_iiwa_status.num_joints),
      Eigen::Map<const VectorXd>(
          msg_iiwa_status.joint_velocity_estimated.data(),
          msg_iiwa_status.num_joints),
      Eigen::Map<const VectorXd>(msg_iiwa_status.joint_torque_external.data(),
                                 msg_iiwa_status.num_joints));
  Command c;

  if (plan) {
    plan->Step(s, control_period_seconds_,
               state_machine_->GetCurrentPlanUpTime(t_now), &c);
  } else if (state_machine_->get_state_type() ==
             PlanManagerStateTypes::kStateIdle) {
    c.q_cmd = state_machine_->get_iiwa_position_command_idle();
    c.tau_cmd = Eigen::VectorXd::Zero(msg_iiwa_status.num_joints);
  } else {
    // In state INIT or ERROR.
    // Send an empty iiwa command message with utime = -1.
    // Behavior with mock_station_simulation:
    //  throws with error message:
    //   IiwaCommandReceiver expected num_joints = 7, but received 0.
    // TODO: confirm the behavior of drake-iiwa-driver (real robot).
    //  What I think will happen:
    //  throws if msg_iiwa_command.num_joints != 7, unless msg_iiwa_command
    //  .utime == -1, in which case no command is sent to the robot.
    msg_iiwa_command = lcmt_iiwa_command();
    msg_iiwa_command.utime = -1;
    return;
  }

  // Check command for error.
  if (!state_machine_->CommandHasError(s, c)) {
    const int num_joints = msg_iiwa_status.num_joints;
    msg_iiwa_command.num_joints = num_joints;
    msg_iiwa_command.num_torques = num_joints;
    msg_iiwa_command.utime = msg_iiwa_status.utime;
    msg_iiwa_command.joint_position.resize(num_joints);
    msg_iiwa_command.joint_torque.resize(num_joints);
    for (int i = 0; i < num_joints; i++) {
      msg_iiwa_command.joint_position[i] = c.q_cmd[i];
      msg_iiwa_command.joint_torque[i] = c.tau_cmd[i];
    }
  }
}

void IiwaPlanManagerSystem::PrintCurrentState(
    const drake::systems::Context<double> &context,
    drake::systems::DiscreteValues<double>*) const {
  state_machine_->PrintCurrentState(context.get_time());
}
