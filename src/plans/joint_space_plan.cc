#include "joint_space_plan.h"

void JointSpacePlan::Step(const State &, double control_period, double t,
                          Command *cmd) const {
  cmd->q_cmd = q_traj_.value(t);
  cmd->tau_cmd = Eigen::VectorXd::Zero(q_traj_.rows());
}
