#include "joint_space_plan.h"

void JointSpacePlan::Step(const State &state,
                          double control_period,
                          double t,
                          Command *cmd) const {
  cmd->q_cmd = q_traj_.value(t);
  cmd->tau_cmd = Eigen::VectorXd::Zero(plant_->num_positions());
}