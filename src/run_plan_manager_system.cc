#include "plan_manager_system/iiwa_plan_manager_hardware_interface.h"

int main() {
  auto plan_manger = IiwaPlanManagerHardwareInterface(0.005);
  plan_manger.SaveGraphvizStringToFile();
  plan_manger.Run(1.0);
  return 0;
}
