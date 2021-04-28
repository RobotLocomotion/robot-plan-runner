#include "drake_lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_plan.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"

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
  auto sub = lcm_status_command_->subscribe(
      config_["lcm_status_channel"].as<std::string>(),
      &IiwaPlanManager::HandleIiwaStatus, this);
  sub->setQueueCapacity(1);
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
  TimePoint current_start_time = std::chrono::high_resolution_clock::now();
  TimePoint next_start_time(current_start_time);
  const auto interval = 1000ms;

  while (true) {
    current_start_time = std::chrono::high_resolution_clock::now();
    next_start_time = current_start_time + interval;
    double t_now_seconds = std::chrono::duration_cast<DoubleSeconds>(
                               current_start_time.time_since_epoch())
                               .count();
    state_machine_->PrintCurrentState(t_now_seconds);
    std::this_thread::sleep_until(next_start_time);
  }
}

void IiwaPlanManager::ReceivePlans() {
  const std::string addr_prefix("tcp://*:");
  zmq::socket_t plan_server(zmq_ctx_, zmq::socket_type::rep);
  plan_server.bind(addr_prefix + config_["zmq_socket_plan"].as<std::string>());

  zmq::socket_t status_publisher(zmq_ctx_, zmq::socket_type::pub);
  status_publisher.bind(addr_prefix +
                        config_["zmq_socket_status"].as<std::string>());

  const auto channel_name_string =
      config_["status_channel_name"].as<std::string>();

  const auto status_update_period = std::chrono::milliseconds(static_cast<int>(
      config_["staus_update_period_seconds"].as<double>() * 1000));

  while (true) {
    zmq::message_t plan_msg;
    // Blocks until a plan msg is received.
    auto res = plan_server.recv(plan_msg, zmq::recv_flags::none);
    DRAKE_THROW_UNLESS(res.has_value());

    res = plan_server.send(zmq::str_buffer("plan_received"),
                           zmq::send_flags::none);
    DRAKE_THROW_UNLESS(res.has_value());

    drake::lcmt_robot_plan plan_lcm_msg;
    plan_lcm_msg.decode(plan_msg.data(), 0, plan_msg.size());
    auto plan = plan_factory_->MakePlan(plan_lcm_msg);

    {
      std::lock_guard<std::mutex> lock(mutex_state_machine_);
      state_machine_->QueueNewPlan(std::move(plan));
    }

    // Handle received plan.
    TimePoint current_start_time = std::chrono::high_resolution_clock::now();
    TimePoint next_start_time(current_start_time);
    while (true) {
      current_start_time = std::chrono::high_resolution_clock::now();
      next_start_time = current_start_time + status_update_period;

      std::string reply_msg(channel_name_string + " ");
      const auto current_state = state_machine_->get_state_type();
      switch (current_state) {
      case PlanManagerStateTypes::kStateRunning: {
        reply_msg += "running";
        break;
      }
      case PlanManagerStateTypes::kStateIdle: {
        reply_msg += "success";
        break;
      }
      case PlanManagerStateTypes::kStateError: {
        reply_msg += "error";
        break;
      }
      case PlanManagerStateTypes::kStateInit: {
        reply_msg += "discarded";
        break;
      }
      }

      zmq::message_t reply(reply_msg.size());
      memcpy(reply.data(), reply_msg.data(), reply_msg.size());
      status_publisher.send(reply, zmq::send_flags::none);

      if (current_state != PlanManagerStateTypes::kStateRunning) {
        break;
      }
      std::this_thread::sleep_until(next_start_time);
    }
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
