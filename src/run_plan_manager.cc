#include "iiwa_plan_manager.h"
#include <iostream>

int main(int argc, char **argv) {
  std::string filename;
  if (argc < 2) {
    filename = "../config/default.yaml";
  } else {
    filename = argv[1];
  }

  YAML::Node config;
  config = YAML::LoadFile(filename);

  IiwaPlanManager pm(config);
  pm.Run();

  return 0;
}