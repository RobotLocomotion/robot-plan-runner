#include <fstream>
#include <numeric>

#include "drake/systems/lcm/lcm_interface_system.h"

#include "iiwa_plan_manager_hardware_interface.h"

IiwaPlanManagerHardwareInterface::IiwaPlanManagerHardwareInterface(
    const YAML::Node &config) {
  owned_lcm_ = std::make_unique<drake::lcm::DrakeLcm>();

  drake::systems::DiagramBuilder<double> builder;

  // PlanManagerSystem.
  plan_manager_ = builder.template AddSystem<IiwaPlanManagerSystem>();
  diagram_ = builder.Build();

  // A standalone subscriber to IIWA_STATUS, used for lcm-driven loop.
  status_sub_ =
      std::make_unique<drake::lcm::Subscriber<drake::lcmt_iiwa_status>>(
          owned_lcm_.get(), config["lcm_status_channel"].as<std::string>());
}

[[noreturn]] void IiwaPlanManagerHardwareInterface::Run() {
  auto context_diagram = diagram_->CreateDefaultContext();
  auto &context_manager =
      plan_manager_->GetMyMutableContextFromRoot(context_diagram.get());

  // Wait for the first IIWA_STATUS message.
  drake::log()->info("Waiting for first lcmt_iiwa_status");
  drake::lcm::LcmHandleSubscriptionsUntil(
      owned_lcm_.get(), [&]() { return status_sub_->count() > 0; });
  auto &status_value = plan_manager_->get_iiwa_status_input_port().FixValue(
      &context_manager, status_sub_->message());
  const double t_start = status_sub_->message().utime * 1e-6;

  drake::log()->info("Controller started");
  while (true) {
    status_sub_->clear();
    // Wait for an IIWA_STATUS message.
    drake::lcm::LcmHandleSubscriptionsUntil(
        owned_lcm_.get(), [&]() { return status_sub_->count() > 0; });

    // Update diagram context.
    status_value.GetMutableData()->set_value(status_sub_->message());
    const double t = status_sub_->message().utime * 1e-6;
    context_diagram->SetTime(t - t_start);

    // Compute command message and publish.
    auto iiwa_cmd_msg = plan_manager_->get_iiwa_command_output_port()
                            .Eval<drake::lcmt_iiwa_command>(context_manager);
    drake::lcm::Publish(owned_lcm_.get(), "IIWA_COMMAND", iiwa_cmd_msg);
  }
}

void IiwaPlanManagerHardwareInterface::SaveGraphvizStringToFile(
    const std::string &file_name) {
  if (diagram_) {
    std::ofstream out(file_name);
    out << diagram_->GetGraphvizString();
    out.close();
  }
}
