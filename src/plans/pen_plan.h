#include <thread>

#include "plans/plan_base.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_robot_state.hpp"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/lcm/lcm_messages.h"
#include <Eigen/Dense>

/*
 * Frame hierarchy:
 * L7 --> T --> C
 * L7 is the frame of link 7 of the IIWA.
 * T is fixed relative to L7, called the tool frame.
 * C is fixed to the tool handle, called the compliant frame.
 * X_L7T is fixed and hard coded.
 * X_WL7_0 is the pose of frame L7 at the beginning of the plan.
 * Frame Cd (Compliant-desired) is initialized to be identical to T when the
 *  plan is constructed: X_WT_0 == X_WCd
 * The plan subscribes to Relative Pose Estimator and receives X_TC.
 * The goal is to have T track Td (Tool_desired), so that
 *  X_WTd * X_TC = X_WCd.
 *  Also publishes X_WC on the LCM channel "X_WC".
 */
class PenPlan : public PlanBase {
 public:
  PenPlan(drake::trajectories::PiecewiseQuaternionSlerp<double> quat_traj,
               drake::trajectories::PiecewisePolynomial<double> xyz_traj,
               const drake::multibody::MultibodyPlant<double> *plant,
               double control_time_step,
               Eigen::VectorXd nominal_joint_position,
               const double x_desired);

  ~PenPlan() override;

  void Step(const State &state, double control_period, double t,
            Command *cmd) const override;

  [[nodiscard]] double duration() const override {
    return xyz_traj_.end_time() - xyz_traj_.start_time();
  };            
 private:
  // Relative transform between the frame L7 and the frame tool frame T.
  const drake::math::RigidTransformd X_L7T_ = drake::math::RigidTransformd(
      drake::math::RollPitchYawd(Eigen::Vector3d(0, -M_PI / 2, 0)),
      Eigen::Vector3d(0, 0, 0.255));
  const drake::multibody::Frame<double>& frame_L7_;

  // Nominal trajectories.
  const drake::trajectories::PiecewiseQuaternionSlerp<double> quat_traj_;
  const drake::trajectories::PiecewisePolynomial<double> xyz_traj_;

  // DiffIk
  const Eigen::Array3d kp_translation_{100, 100, 100};
  const Eigen::Array3d kp_rotation_{50, 50, 50};
  std::unique_ptr<drake::systems::Context<double>> plant_context_;  
  
  std::unique_ptr<
      drake::manipulation::planner::DifferentialInverseKinematicsParameters>
      params_;  

  // Relative pose estimator subscription.
  void PoseIo() const;
  std::unique_ptr<drake::lcm::DrakeLcm> owned_lcm_;
  std::unique_ptr<drake::lcm::Subscriber<drake::lcmt_robot_state>> rpe_sub_;
  std::thread sub_thread_;
  std::atomic<bool> stop_flag_{false};
  mutable std::mutex mutex_rpe_;
  mutable drake::math::RigidTransformd X_TC_;
  mutable drake::math::RigidTransformd X_WT_;

  // Desired task specification.
  double x_TC_desired_;
  mutable double x_TC_now_;
  mutable double x_previous_;  
  double kp_;
  bool controller_active_;
};
