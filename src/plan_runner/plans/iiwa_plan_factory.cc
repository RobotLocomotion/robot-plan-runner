#include "iiwa_plan_factory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

#include "joint_space_plan.h"

using std::cout;
using std::endl;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::VectorXd;

std::unique_ptr<PlanBase>
IiwaPlanFactory::MakePlan(const drake::lcmt_robot_plan &msg_plan) const {
  // TODO: replace this with a better test and use an lcm type with an
  //  enum for plan types.
  if (msg_plan.plan.at(0).joint_name.at(0) == "iiwa_joint_0") {
    return MakeJointSpacePlan(msg_plan);
  }
  throw std::runtime_error("error in plan lcm message.");
}

std::unique_ptr<PlanBase> IiwaPlanFactory::MakeJointSpacePlan(
    const drake::lcmt_robot_plan &msg_plan) {
  int n_knots = msg_plan.num_states;
  int n_q = msg_plan.plan.at(0).num_joints;
  MatrixXd q_knots(n_q, n_knots);
  VectorXd t_knots(n_knots);

  for (int t = 0; t < n_knots; t++) {
    t_knots[t] = static_cast<double>(msg_plan.plan.at(t).utime) / 1e6;
    for (int i = 0; i < n_q; i++) {
      q_knots(i, t) = msg_plan.plan.at(t).joint_position.at(i);
    }
  }

  auto q_traj =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          t_knots, q_knots, VectorXd::Zero(n_q), VectorXd::Zero(n_q));
  return std::make_unique<JointSpacePlan>(std::move(q_traj), nullptr);
}
