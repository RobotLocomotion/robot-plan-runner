#include "joint_space_trajectory_plan.h"

void JointSpaceTrajectoryPlan::Step(const State &, double control_period,
                                    double t, Command *cmd) const {
  cmd->q_cmd = q_traj_.value(t);
  cmd->tau_cmd = Eigen::VectorXd::Zero(q_traj_.rows());
}
