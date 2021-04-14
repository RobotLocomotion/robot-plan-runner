#include "plan_manager.h"

int main() {
  IiwaPlanManager pm(0.005);
  pm.Run();

  return 0;
}