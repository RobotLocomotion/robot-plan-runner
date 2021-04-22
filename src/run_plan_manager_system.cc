#include "plan_manager_system/iiwa_plan_manager_hardware_interface.h"
#include <iostream>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {
  std::string filename;
  if (argc < 2) {
    filename = "../config/default.yaml";
  } else {
    filename = argv[1];
  }

  YAML::Node config;
  config = YAML::LoadFile(filename);

  auto plan_manger = IiwaPlanManagerHardwareInterface(config);
  plan_manger.SaveGraphvizStringToFile();
  plan_manger.Run(1.0);
  return 0;
}
