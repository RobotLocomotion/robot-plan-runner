#include "state_idle.h"

PlanManagerStateBase* StateIdle::instance_ = nullptr;
PlanManagerStateBase* StateIdle::Instance() {
  if (!instance_) {
    instance_ = new StateIdle();
  }
  return instance_;
}