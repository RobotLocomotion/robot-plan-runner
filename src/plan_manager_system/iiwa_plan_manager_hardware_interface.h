#pragma once

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/framework/diagram_builder.h"

#include "plan_manager_system/iiwa_plan_manager_system.h"

class IiwaPlanManagerHardwareInterface {
public:
  explicit IiwaPlanManagerHardwareInterface(const YAML::Node &config);

  [[noreturn]] void Run();

  /*
   * Saves the graphviz string which describes this system to a file.
   */
  void SaveGraphvizStringToFile(
      const std::string &file_name = "system_graphviz_string.txt");

private:
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  std::unique_ptr<drake::lcm::DrakeLcm> owned_lcm_;
  std::unique_ptr<drake::lcm::Subscriber<drake::lcmt_iiwa_status>> status_sub_;
  IiwaPlanManagerSystem *plan_manager_{nullptr};
};
