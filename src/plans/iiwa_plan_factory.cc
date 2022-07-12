#include "iiwa_plan_factory.h"

#include <iostream>

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

  const auto &iiwa_path = config_["robot_sdf_path"].as<std::string>();
  parser.AddModelFromFile(drake::FindResourceOrThrow(iiwa_path));
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
  int n_knots = msg_plan.num_states - 1;

  // X_ET.
  const auto& q_xyz_ET = msg_plan.plan[n_knots].joint_position;
  const auto Q_ET = Quaterniond(q_xyz_ET[0], q_xyz_ET[1], q_xyz_ET[2],
                                q_xyz_ET[3]);
  const auto p_EoTo_E = Eigen::Vector3d(q_xyz_ET[4], q_xyz_ET[5], q_xyz_ET[6]);
  const auto X_ET = drake::math::RigidTransform<double>(Q_ET, p_EoTo_E);

  // trajectory.
  vector<double> t_knots(n_knots);
  vector<Quaterniond> quat_knots(n_knots);
  vector<MatrixXd> xyz_knots(n_knots, MatrixXd(3, 1));

  for (int t = 0; t < n_knots; t++) {
    // Store time
    t_knots[t] = static_cast<double>(msg_plan.plan.at(t).utime) / 1e6;
    // Store quaternions
    const auto& q_xyz = msg_plan.plan.at(t).joint_position;
    quat_knots[t] = Quaterniond(q_xyz[0], q_xyz[1], q_xyz[2], q_xyz[3]);

    // Store xyz positions.
    // TODO(terry-suh): Change this to a better implementation once Taskspace
    // gets their own LCM messages.
    for (int i = 0; i < 3; i++) {
      xyz_knots[t](i) = q_xyz[4 + i];
    }
  }

  auto quat_traj = PiecewiseQuaternionSlerp<double>(t_knots, quat_knots);

  // Use first order hold instead of trajectories. We want to avoid smoothing
  // non-smooth trajectories if the user commands so.
  // TODO(terry-suh): Should this be an option in the config?
  auto xyz_traj =
      PiecewisePolynomial<double>::FirstOrderHold(t_knots, xyz_knots);

  // Get EE frame.
  const auto &frame_E =
      plant_->GetFrameByName(config_["robot_ee_body_name"].as<std::string>());

  vector<double> nominal_joint_vector = 
    config_["robot_nominal_joint"].as<vector<double>>();

  Eigen::VectorXd nominal_joint= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
    nominal_joint_vector.data(), nominal_joint_vector.size());


  return std::make_unique<TaskSpaceTrajectoryPlan>(
      std::move(quat_traj), std::move(xyz_traj), X_ET, plant_.get(), frame_E,
      config_["control_period"].as<double>(), nominal_joint);
}
