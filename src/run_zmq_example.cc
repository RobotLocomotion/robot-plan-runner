#include <iostream>
#include <string>

#include "drake_lcmtypes/drake/lcmt_robot_plan.hpp"
#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"
#include <yaml-cpp/yaml.h>
#include <zmq.hpp>

#include "plans/iiwa_plan_factory.h"

using std::cout;
using std::endl;

void PrintRobotState(const drake::lcmt_robot_state &state_msg) {
  cout << "t: " << static_cast<double>(state_msg.utime) / 1e6 << endl;
  cout << "num joints: " << state_msg.num_joints << endl;
  for (int i = 0; i < state_msg.num_joints; i++) {
    cout << state_msg.joint_name[i] << " " << state_msg.joint_position[i]
         << endl;
  }
  cout << "---------------------------------------" << endl;
}

int main() {
  zmq::context_t ctx;
  zmq::socket_t sock(ctx, zmq::socket_type::push);
  zmq::socket_t socket(ctx, ZMQ_REP);
  socket.bind("tcp://*:5555");

  std::string config_filename = "../config/default.yaml";
  YAML::Node config;
  config = YAML::Load(config_filename);

  IiwaPlanFactory plan_factory(config);
  State s;
  Command c;

  drake::lcmt_robot_plan plan_lcm_msg;
  while (true) {
    zmq::message_t plan_msg;
    auto res = socket.recv(plan_msg, zmq::recv_flags::none);
    if (!res.has_value()) {
      throw std::runtime_error("Receiving plan message failed.");
    }

    cout << "received msg" << endl;
    plan_lcm_msg.decode(plan_msg.data(), 0, plan_msg.size());
    cout << "plan num states " << plan_lcm_msg.num_states << endl;
    for (const auto &state_msg : plan_lcm_msg.plan) {
      PrintRobotState(state_msg);
    }

    cout << "compare strings "
         << (plan_lcm_msg.plan[0].joint_name[0] == "iiwa_joint_0") << endl;

    auto plan = plan_factory.MakePlan(plan_lcm_msg, config);
    plan->Step(s, 0.005, 0.876, &c);
    cout << "q_cmd " << c.q_cmd.transpose() << endl;
    cout << "tau_cmd " << c.tau_cmd.transpose() << endl;

    std::string reply_msg("success");
    zmq::message_t reply(reply_msg.size());
    memcpy(reply.data(), reply_msg.data(), reply_msg.size());
    socket.send(reply, zmq::send_flags::none);
  }
}
