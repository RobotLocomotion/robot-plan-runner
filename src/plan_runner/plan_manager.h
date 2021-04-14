#pragma once
#include <mutex>
#include <thread>
#include <unordered_map>
#include "lcm/lcm-cpp.hpp"
#include "drake_lcmtypes/drake/lcmt_iiwa_status.hpp"

#include "plan_manager_state_machine.h"


class IiwaPlanManager {
 public:
  IiwaPlanManager(double control_period);
  ~IiwaPlanManager();
  void Run();

 private:
  const double control_period_;
  mutable std::mutex mutex_state_machine_;
  std::unique_ptr<PlanManagerStateMachine> state_machine_;
  std::unordered_map<std::string, std::thread> threads_;

  // Iiwa status + command thread.
  std::unique_ptr<lcm::LCM> lcm_status_command_;
  drake::lcmt_iiwa_status iiwa_status_msg_;
  std::mutex mutex_iiwa_status_;
  void CalcCommandFromStatus();
  void HandleIiwaStatus(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel,
                        const drake::lcmt_iiwa_status* status_msg);

  // Printing thread.
  void PrintStateMachineStatus() const;

  // Robot plans thread.

};

