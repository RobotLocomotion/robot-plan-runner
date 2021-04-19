#include <fstream>
#include <numeric>

#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"

#include "iiwa_plan_manager_hardware_interface.h"

IiwaPlanManagerHardwareInterface::IiwaPlanManagerHardwareInterface(
    double control_period_seconds) {
  owned_lcm_ = std::make_unique<drake::lcm::DrakeLcm>();

  drake::systems::DiagramBuilder<double> builder;
  auto lcm =
      builder.template AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
          owned_lcm_.get());

  // PlanManagerSystem.
  auto plan_manager =
      builder.template AddSystem<IiwaPlanManagerSystem>(control_period_seconds);

  // Subscribe to IIWA_STATUS.
  auto sub_iiwa_status = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));
  builder.Connect(sub_iiwa_status->get_output_port(),
                  plan_manager->get_iiwa_status_input_port());

  // Subscribe to ROBOT_PLAN.
  auto sub_robot_plans = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_robot_plan>(
          "ROBOT_PLAN", lcm));
  builder.Connect(sub_robot_plans->get_output_port(),
                  plan_manager->get_robot_plan_input_port());

  // Publish iiwa command.
  auto iiwa_command_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm, control_period_seconds));
  builder.Connect(plan_manager->GetOutputPort("lcmt_iiwa_command"),
                  iiwa_command_pub->get_input_port());

  diagram_ = builder.Build();
}

void IiwaPlanManagerHardwareInterface::Run(double realtime_rate) {
  drake::systems::Simulator<double> sim(*diagram_);
  sim.set_publish_every_time_step(false);
  sim.set_target_realtime_rate(realtime_rate);

  sim.AdvanceTo(std::numeric_limits<double>::infinity());
}

void IiwaPlanManagerHardwareInterface::SaveGraphvizStringToFile(
    const std::string &file_name) {
  if (diagram_) {
    std::ofstream out(file_name);
    out << diagram_->GetGraphvizString();
    out.close();
  }
}