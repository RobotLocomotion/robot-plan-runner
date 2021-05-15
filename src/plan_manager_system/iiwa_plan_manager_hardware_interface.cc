#include <fstream>
#include <numeric>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "iiwa_plan_manager_hardware_interface.h"

IiwaPlanManagerHardwareInterface::IiwaPlanManagerHardwareInterface(
    const YAML::Node &config) {

  double control_period_seconds = config["control_period"].as<double>();

  owned_lcm_ = std::make_unique<drake::lcm::DrakeLcm>();

  drake::systems::DiagramBuilder<double> builder;
  auto lcm =
      builder.template AddSystem<drake::systems::lcm::LcmInterfaceSystem>(
          owned_lcm_.get());

  // PlanManagerSystem.
  auto plan_manager =
      builder.template AddSystem<IiwaPlanManagerSystem>();

  // Subscribe to IIWA_STATUS.
  auto sub_iiwa_status = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          config["lcm_status_channel"].as<std::string>(), lcm));
  builder.Connect(sub_iiwa_status->get_output_port(),
                  plan_manager->get_iiwa_status_input_port());


  // Publish iiwa command.
  auto iiwa_command_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          config["lcm_command_channel"].as<std::string>(), lcm,
          control_period_seconds));
  builder.Connect(plan_manager->get_iiwa_command_output_port(),
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
