#include "iiwa_plan_manager_system.h"
#include "drake/common/value.h"

using drake::lcmt_iiwa_command;
using drake::lcmt_iiwa_status;
using drake::lcmt_robot_plan;
using drake::systems::BasicVector;

IiwaPlanManagerSystem::IiwaPlanManagerSystem(double control_period_seconds)
    : control_period_seconds_(control_period_seconds),
      plant_(plan_factory_.get_plant()) {
  set_name("IiwaPlanManagerSystem");
  // Abstract state and update events.
  // TODO: compute output in the output port callback function, as publish
  //  rate is driven by lcm publisher system.
  abstract_state_idx_ =
      DeclareAbstractState(*drake::AbstractValue::Make(lcmt_iiwa_command()));
  DeclarePeriodicUnrestrictedUpdateEvent(
      control_period_seconds_, 0, &IiwaPlanManagerSystem::UpdateIiwaCommand);
  DeclarePeriodicUnrestrictedUpdateEvent(
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

  // Output ports.
  DeclareAbstractOutputPort("lcmt_iiwa_command",
                            &IiwaPlanManagerSystem::CopyStateOut);
}

void IiwaPlanManagerSystem::UpdateIiwaCommand(
    const drake::systems::Context<double> &context,
    drake::systems::State<double> *state) const {
  const auto &msg_iiwa_status =
      get_iiwa_status_input_port().Eval<lcmt_iiwa_status>(context);
  const auto &msg_robot_plan =
      get_robot_plan_input_port().Eval<lcmt_robot_plan>(context);
  auto &msg_iiwa_command =
      state->get_mutable_abstract_state<lcmt_iiwa_command>(abstract_state_idx_);

  if (msg_iiwa_status.num_joints != 0) {
    state_machine_.receive_new_status_msg();
  }
}

void IiwaPlanManagerSystem::PrintCurrentState(
    const drake::systems::Context<double> &context,
    drake::systems::State<double> *) const {
  std::cout << "t = " << context.get_time() << ". ";
  state_machine_.PrintCurrentState();
}

void IiwaPlanManagerSystem::CopyStateOut(
    const drake::systems::Context<double> &context,
    lcmt_iiwa_command *cmd) const {
  *cmd = context.get_abstract_state<lcmt_iiwa_command>(abstract_state_idx_);
}