#include "plan_manager_state_machine.h"
#include "state_init.h"

PlanManagerStateMachine::PlanManagerStateMachine() {
  // Initialize to state INIT.
  state_ = StateInit::Instance();
}




