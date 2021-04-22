#pragma once
#include <mutex>
#include <thread>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include "drake_lcmtypes/drake/lcmt_iiwa_status.hpp"
#include "lcm/lcm-cpp.hpp"

#include "plans/iiwa_plan_factory.h"
#include "state_machine/plan_manager_state_machine.h"

class IiwaPlanManager {
public:
  IiwaPlanManager(YAML::Node config);
  ~IiwaPlanManager();
  void Run();

private:
  double control_period_seconds_;
  mutable std::mutex mutex_state_machine_;
  std::unique_ptr<PlanManagerStateMachine> state_machine_;
  std::unordered_map<std::string, std::thread> threads_;
  std::unique_ptr<IiwaPlanFactory> plan_factory_;

  YAML::Node config_;

  // Iiwa status + command thread.
  std::unique_ptr<lcm::LCM> lcm_status_command_;
  drake::lcmt_iiwa_status iiwa_status_msg_;
  std::mutex mutex_iiwa_status_;
  void CalcCommandFromStatus();
  void HandleIiwaStatus(const lcm::ReceiveBuffer *rbuf,
                        const std::string &channel,
                        const drake::lcmt_iiwa_status *status_msg);

  // Printing thread.
  void PrintStateMachineStatus() const;

  // Robot plans thread.
  void ReceivePlans();
};
