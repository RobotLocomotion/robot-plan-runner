#include "iiwa_plan_factory.h"
#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/multibody/parsing/parser.h"

#include "joint_space_trajectory_plan.h"
#include "task_space_trajectory_plan.h"

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

IiwaPlanFactory::IiwaPlanFactory(const YAML::Node &config) : config_(config) {
  plant_ = std::make_unique<drake::multibody::MultibodyPlant<double>>(1e-3);
  auto parser = drake::multibody::Parser(plant_.get());

  std::string iiwa_path = 
    config_["robot_sdf_path"].as<std::string>();
  iiwa_path = drake::FindResourceOrThrow(iiwa_path);
  parser.AddModelFromFile(iiwa_path);
  plant_->WeldFrames(
      plant_->world_frame(),
      plant_->GetFrameByName(config_["robot_baselink_name"].as<std::string>()));
  plant_->Finalize();
}

std::unique_ptr<PlanBase>
IiwaPlanFactory::MakePlan(const drake::lcmt_robot_plan &msg_plan) const {
  // TODO: replace this with a better test and use an lcm type with an
  //  enum for plan types?
  if (msg_plan.plan.at(0).joint_name.at(0) == "iiwa_joint_0") {
    return MakeJointSpaceTrajectoryPlan(msg_plan);
  } else if (msg_plan.plan.at(0).joint_name.at(0) == "qw") {
    return MakeTaskSpaceTrajectoryPlan(msg_plan);
  }
  throw std::runtime_error("error in plan lcm message.");
}

std::unique_ptr<PlanBase> IiwaPlanFactory::MakeJointSpaceTrajectoryPlan(
    const drake::lcmt_robot_plan &msg_plan) const {
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
  return std::make_unique<JointSpaceTrajectoryPlan>(std::move(q_traj),
                                                    plant_.get());
}

std::unique_ptr<PlanBase> IiwaPlanFactory::MakeTaskSpaceTrajectoryPlan(
    const drake::lcmt_robot_plan &msg_plan) const {

  int n_knots = msg_plan.num_states;
  vector<double> t_knots(n_knots);
  vector<Quaterniond> quat_knots(n_knots);
  vector<MatrixXd> xyz_knots(n_knots, MatrixXd(3, 1));

  for (int t = 0; t < n_knots; t++) {
    // Store time
    t_knots[t] = static_cast<double>(msg_plan.plan.at(t).utime) / 1e6;
    // Store quaternions
    auto quat = msg_plan.plan.at(t).joint_position;
    quat_knots[t] = Quaterniond(quat.at(0), quat.at(1), quat.at(2), quat.at(3));

    // Store xyz positions.
    // TODO(terry-suh): Change this to a better implementation once Taskspace
    // gets their own LCM messages.
    for (int i = 0; i < 3; i++) {
      xyz_knots[t](i) = msg_plan.plan.at(t).joint_position.at(4 + i);
    }
  }

  auto quat_traj = PiecewiseQuaternionSlerp<double>(t_knots, quat_knots);
  auto xyz_traj =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          t_knots, xyz_knots, VectorXd::Zero(3), VectorXd::Zero(3));

  return std::make_unique<TaskSpaceTrajectoryPlan>(
      std::move(quat_traj), std::move(xyz_traj), plant_.get(),
      config_["robot_ee_body_name"].as<std::string>());
}
