#pragma once

#include <cmath>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/context.h"
#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"
#include "lcm/lcm-cpp.hpp"

#include "plans/plan_base.h"

class RigidTrajectoryPlan : public PlanBase {
public:
  RigidTrajectoryPlan(
      drake::trajectories::PiecewiseQuaternionSlerp<double> quat_traj,
      drake::trajectories::PiecewisePolynomial<double> xyz_traj,
      drake::math::RigidTransformd X_ET,
      const drake::multibody::MultibodyPlant<double> *plant,
      const drake::multibody::Frame<double> &frame_E,
      double control_time_step,
      const Eigen::Vector3d Krpy, const Eigen::Vector3d Drpy,
      const Eigen::Vector3d Kxyz, const Eigen::Vector3d Dxyz,      
      const Eigen::Vector3d taud_T, const Eigen::Vector3d fd_W)
      : PlanBase(plant), quat_traj_(std::move(quat_traj)),
        X_ET_(std::move(X_ET)), xyz_traj_(std::move(xyz_traj)),
        frame_E_(frame_E), lcm_(std::make_unique<lcm::LCM>()),
        Krpy_(Krpy), Drpy_(Drpy), Kxyz_(Kxyz), Dxyz_(Dxyz), 
        taud_T_(taud_T), fd_W_(fd_W) {

    params_ = std::make_unique<
        drake::manipulation::planner::DifferentialInverseKinematicsParameters>(
        plant_->num_positions(), plant_->num_velocities());
    params_->set_timestep(control_time_step);

    plant_context_ = plant_->CreateDefaultContext();

    // Initialize LCM.
    is_running_ = true;
    threads_["subscribe_force_torque"] = 
      std::thread(&RigidTrajectoryPlan::SubscribeForceTorque, this);
    threads_["subscribe_velocity"] = 
      std::thread(&RigidTrajectoryPlan::SubscribeVelocity, this);      
    threads_["subscribe_pose"] = 
      std::thread(&RigidTrajectoryPlan::SubscribePose, this);

    // Initialize FT / relative velocity to zero.
    F_TC_ = Eigen::VectorXd::Zero(6);
    V_TC_ = Eigen::VectorXd::Zero(6);
    X_TC_ = drake::math::RigidTransformd();

    std::cout << taud_T_ << std::endl;
    std::cout << fd_W_ << std::endl; 
    std::cout << Krpy_ << std::endl;
    std::cout << Drpy_ << std::endl;
    std::cout << Kxyz_ << std::endl;
    std::cout << Dxyz_ << std::endl;
           
  }

  ~RigidTrajectoryPlan() override;

  void SubscribePose();
  void SubscribeForceTorque();
  void SubscribeVelocity();

  void HandleForceTorqueStatus(
    const lcm::ReceiveBuffer *, const std::string & channel,
    const drake::lcmt_robot_state *status_msg
  );

  void HandleVelocityStatus(
    const lcm::ReceiveBuffer *, const std::string & channel,
    const drake::lcmt_robot_state *status_msg
  );  

  void HandlePoseStatus(
    const lcm::ReceiveBuffer *, const std::string & channel,
    const drake::lcmt_robot_state *status_msg
  );

  void Step(const State &state, double control_period, double t,
            Command *cmd) const override;

  [[nodiscard]] double duration() const override {
    return xyz_traj_.end_time() - xyz_traj_.start_time();
  };

private:
  const drake::trajectories::PiecewiseQuaternionSlerp<double> quat_traj_;
  const drake::trajectories::PiecewisePolynomial<double> xyz_traj_;
  const drake::math::RigidTransformd X_ET_;

  // frame of end-effector body + offset.
  const drake::multibody::Frame<double> &frame_E_;

  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  std::unique_ptr<
      drake::manipulation::planner::DifferentialInverseKinematicsParameters>
      params_;

  // LCM related threads.
  std::unique_ptr<lcm::LCM> lcm_;
  std::unordered_map<std::string, std::thread> threads_;
  std::atomic<bool> is_running_;

  // LCM Updated Quantities.
  Eigen::VectorXd F_TC_; // force-torque received from lcm.
  Eigen::VectorXd V_TC_; // spatial velocity received from lcm.
  drake::math::RigidTransformd X_TC_; // spatial pose received from lcm.  

  // desired torques and forces.
  Eigen::Vector3d taud_T_; // torque seen from the tool frame.
  Eigen::Vector3d fd_W_; // force seen from the world frame.

  // Bushing quantities
  Eigen::Vector3d Krpy_;
  Eigen::Vector3d Drpy_;
  Eigen::Vector3d Kxyz_;
  Eigen::Vector3d Dxyz_;    

};
