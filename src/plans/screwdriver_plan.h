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
class ScrewdriverPlan : public PlanBase {
 public:
  ScrewdriverPlan(drake::trajectories::PiecewisePolynomial<double> q_traj,
               const drake::multibody::MultibodyPlant<double> *plant,
               double control_time_step);

  ~ScrewdriverPlan() override;

  void Step(const State &state, double control_period, double t,
            Command *cmd) const override;

  [[nodiscard]] double duration() const override {
    return q_traj_.end_time() - q_traj_.start_time();
  };            

 private:
  // Nominal trajectories.
  const drake::trajectories::PiecewisePolynomial<double> q_traj_;

  // DiffIk
  std::unique_ptr<drake::systems::Context<double>> plant_context_;  
  

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
  double yaw_TC_desired_;
  mutable double yaw_TC_now_;
  mutable double q_set_;
  mutable bool controller_active_;
};
