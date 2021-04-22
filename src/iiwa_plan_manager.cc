#include "drake_lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_plan.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"
#include <zmq.hpp>

#include "iiwa_plan_manager.h"
#include "plans/plan_base.h"

using Eigen::VectorXd;
using std::cout;
using std::endl;

IiwaPlanManager::IiwaPlanManager(YAML::Node config)
    : config_(std::move(config)),
      control_period_seconds_(config["control_period"].as<double>()) {
  double t_now_seconds =
      std::chrono::duration_cast<DoubleSeconds>(
          std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();
  state_machine_ =
      std::make_unique<PlanManagerStateMachine>(t_now_seconds, config_);
  plan_factory_ = std::make_unique<IiwaPlanFactory>(config_);

  // TODO(terry-suh): make another method here to check for errors in the config
  // file. are all the required fields there?
}

IiwaPlanManager::~IiwaPlanManager() {
  // Wait for all threads to terminate.
  for (auto &a : threads_) {
    if (a.second.joinable()) {
      a.second.join();
    }
  }
}

void IiwaPlanManager::Run() {
  threads_["status_command"] =
      std::thread(&IiwaPlanManager::CalcCommandFromStatus, this);
  threads_["print_status"] =
      std::thread(&IiwaPlanManager::PrintStateMachineStatus, this);
  threads_["receive_plans"] = std::thread(&IiwaPlanManager::ReceivePlans, this);
}

void IiwaPlanManager::CalcCommandFromStatus() {
  lcm_status_command_ = std::make_unique<lcm::LCM>();
  lcm_status_command_->subscribe(
      config_["lcm_status_channel"].as<std::string>(),
      &IiwaPlanManager::HandleIiwaStatus, this);
  while (true) {
    // >0 if a message was handled,
    // 0 if the function timed out,
    // <0 if an error occured.
    if (lcm_status_command_->handleTimeout(10) < 0) {
      break;
    }
  }
}

void IiwaPlanManager::PrintStateMachineStatus() const {
  using namespace std::chrono_literals;
  while (true) {
    std::this_thread::sleep_for(1000ms);
    double t_now_seconds =
        std::chrono::duration_cast<DoubleSeconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    state_machine_->PrintCurrentState(t_now_seconds);
  }
}

void IiwaPlanManager::ReceivePlans() {
  zmq::context_t ctx;
  zmq::socket_t socket(ctx, ZMQ_REP);
  socket.bind(config_["zmq_socket"].as<std::string>());

  while (true) {
    zmq::message_t plan_msg;
    auto res = socket.recv(plan_msg, zmq::recv_flags::none);

    if (!res.has_value()) {
      throw std::runtime_error("Receiving plan message failed.");
    }

    drake::lcmt_robot_plan plan_lcm_msg;
    plan_lcm_msg.decode(plan_msg.data(), 0, plan_msg.size());
    auto plan = plan_factory_->MakePlan(plan_lcm_msg);

    {
      std::lock_guard<std::mutex> lock(mutex_state_machine_);
      state_machine_->QueueNewPlan(std::move(plan));
    }

    std::string reply_msg;
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      if (state_machine_->get_state_type() ==
          PlanManagerStateTypes::kStateRunning) {
        continue;
      }
      if (state_machine_->get_state_type() ==
          PlanManagerStateTypes::kStateIdle) {
        reply_msg = "success";
        break;
      }
      if (state_machine_->get_state_type() ==
          PlanManagerStateTypes::kStateError) {
        reply_msg = "error";
        break;
      }
      if (state_machine_->get_state_type() ==
          PlanManagerStateTypes::kStateInit) {
        reply_msg = "discarded";
        break;
      }
    }
    zmq::message_t reply(reply_msg.size());
    memcpy(reply.data(), reply_msg.data(), reply_msg.size());
    socket.send(reply, zmq::send_flags::none);
  }
}

void IiwaPlanManager::HandleIiwaStatus(
    const lcm::ReceiveBuffer *, const std::string &channel,
    const drake::lcmt_iiwa_status *status_msg) {
  {
    std::lock_guard<std::mutex> lock(mutex_iiwa_status_);
    iiwa_status_msg_ = *status_msg;
  }
  const PlanBase *plan;
  auto t_now = std::chrono::high_resolution_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_state_machine_);
    state_machine_->ReceiveNewStatusMsg(*status_msg);
    plan = state_machine_->GetCurrentPlan(t_now, *status_msg);
  }

  const int num_joints = iiwa_status_msg_.num_joints;
  // Compute command.
  State s(Eigen::Map<VectorXd>(iiwa_status_msg_.joint_position_measured.data(),
                               num_joints),
          Eigen::Map<VectorXd>(iiwa_status_msg_.joint_velocity_estimated.data(),
                               num_joints),
          Eigen::Map<VectorXd>(iiwa_status_msg_.joint_torque_external.data(),
                               num_joints));
  Command c;
  if (plan) {
    plan->Step(s, control_period_seconds_,
               state_machine_->GetCurrentPlanUpTime(t_now), &c);
  } else if (state_machine_->get_state_type() ==
             PlanManagerStateTypes::kStateIdle) {
    c.q_cmd = state_machine_->get_iiwa_position_command_idle();
    c.tau_cmd = Eigen::VectorXd::Zero(num_joints);
  } else {
    // No commands are sent in state INIT or ERROR.
    return;
  }

  // Check command for error.
  bool command_has_error;
  {
    std::lock_guard<std::mutex> lock(mutex_state_machine_);
    command_has_error = state_machine_->CommandHasError(s, c);
  }
  if (!command_has_error) {
    drake::lcmt_iiwa_command cmd_msg;
    cmd_msg.num_joints = num_joints;
    cmd_msg.num_torques = num_joints;
    cmd_msg.utime = status_msg->utime;
    for (int i = 0; i < num_joints; i++) {
      cmd_msg.joint_position.push_back(c.q_cmd[i]);
      cmd_msg.joint_torque.push_back(c.tau_cmd[i]);
    }
    lcm_status_command_->publish(
        config_["lcm_command_channel"].as<std::string>(), &cmd_msg);
  }
}
