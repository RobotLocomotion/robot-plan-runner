#include <vector>

#include "drake_lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_plan.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"

#include "iiwa_plan_manager.h"
#include "plans/plan_base.h"
#include "robot_plan_runner/lcmt_plan_status.hpp"
#include "robot_plan_runner/lcmt_plan_status_constants.hpp"

using Eigen::VectorXd;
using robot_plan_runner::lcmt_plan_status;
using robot_plan_runner::lcmt_plan_status_constants;
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
  threads_["receive_plans"] =
      std::thread(&IiwaPlanManager::ReceivePlanAndPublishPlanStatus, this);
  threads_["cancel_plans"] = std::thread(&IiwaPlanManager::AbortPlans, this);
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

// Constructs a vector of bytes consisting of the channel name and the
// encoded LCM message separated by a space:
//  (channel_name, ' ', encoded LCM message)
std::vector<uint8_t> PrependLcmMsgWithChannel(const std::string &channel_name,
                                              const lcmt_plan_status &msg) {
  const int data_len = msg.getEncodedSize();
  const int channel_len = channel_name.size();

  std::vector<uint8_t> msg_full_bytes(data_len + channel_len + 1);
  for (size_t i = 0; i < channel_len; i++) {
    msg_full_bytes[i] = channel_name[i];
  }
  msg_full_bytes[channel_len] = ' ';
  msg.encode(msg_full_bytes.data(), channel_len + 1, data_len);

  return msg_full_bytes;
}

void IiwaPlanManager::ReceivePlanAndPublishPlanStatus() {
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
    lcmt_plan_status msg_plan_status;
    msg_plan_status.utime = plan_lcm_msg.utime;
    TimePoint current_start_time = std::chrono::high_resolution_clock::now();
    TimePoint next_start_time(current_start_time);
    while (true) {
      current_start_time = std::chrono::high_resolution_clock::now();
      next_start_time = current_start_time + status_update_period;

      const auto current_state = state_machine_->get_state_type();
      switch (current_state) {
      case PlanManagerStateTypes::kStateRunning: {
        msg_plan_status.status = lcmt_plan_status_constants::RUNNING;
        break;
      }
      case PlanManagerStateTypes::kStateIdle: {
        // TODO: aborted plans and successfully finished plans are not
        //  distinguished, both are designated "FINISHED".
        //  On the other hand, if the client calls abort, it should know that
        //  the plan doesn't finish normally?
        msg_plan_status.status = lcmt_plan_status_constants::FINISHED;
        break;
      }
      case PlanManagerStateTypes::kStateError: {
        msg_plan_status.status = lcmt_plan_status_constants::ERROR;
        break;
      }
      case PlanManagerStateTypes::kStateInit: {
        msg_plan_status.status = lcmt_plan_status_constants::DISCARDED;
        break;
      }
      }
      auto reply_msg =
          PrependLcmMsgWithChannel(channel_name_string, msg_plan_status);
      zmq::message_t reply(reply_msg.begin(), reply_msg.end());
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
  const PlanBase *plan;
  auto t_now = std::chrono::high_resolution_clock::now();
  const int num_joints = (*status_msg).num_joints;
  State s(Eigen::Map<const VectorXd>(
              (*status_msg).joint_position_measured.data(), num_joints),
          Eigen::Map<const VectorXd>(
              (*status_msg).joint_velocity_estimated.data(), num_joints),
          Eigen::Map<const VectorXd>((*status_msg).joint_torque_external.data(),
                                     num_joints));
  Command c;
  bool command_has_error;
  {
    // Lock state machine.
    std::lock_guard<std::mutex> lock(mutex_state_machine_);

    // Get current plan.
    plan = state_machine_->GetCurrentPlan(t_now, *status_msg);
    state_machine_->ReceiveNewStatusMsg(*status_msg);

    // Compute command.
    if (plan) {
      const double t_plan = state_machine_->GetCurrentPlanUpTime(t_now);
      plan->Step(s, control_period_seconds_, t_plan, &c);
    } else if (state_machine_->get_state_type() ==
               PlanManagerStateTypes::kStateIdle) {
      c.q_cmd = state_machine_->get_iiwa_position_command_idle();
      c.tau_cmd = Eigen::VectorXd::Zero(num_joints);
    } else {
      // No commands are sent in state INIT or ERROR.
      return;
    }

    // Check command for error.
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

void IiwaPlanManager::AbortPlans() {
  zmq::socket_t abort_server(zmq_ctx_, zmq::socket_type::rep);
  abort_server.bind("tcp://*:" + config_["zmq_socket_abort"].as<std::string>());

  zmq::message_t msg;
  while (true) {
    auto res = abort_server.recv(msg, zmq::recv_flags::none);
    DRAKE_THROW_UNLESS(res.has_value());

    {
      std::lock_guard<std::mutex> lock(mutex_state_machine_);
      state_machine_->AbortAllPlans();
    }

    res = abort_server.send(zmq::str_buffer("plans_aborted"),
                            zmq::send_flags::none);
    DRAKE_THROW_UNLESS(res.has_value());

    double t_now =
        std::chrono::duration_cast<DoubleSeconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    auto t_up = state_machine_->get_state_machine_up_time(t_now);

    cout << "t = " << t_up << ". [INFO]: all plans have been aborted. "
         << "Returning to [IDLE]." << endl;
  }
}
