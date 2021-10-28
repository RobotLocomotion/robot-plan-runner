#include <thread>

#include "plans/plan_base.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_robot_state.hpp"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"
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
 */
class ChickenHeadPlan : public PlanBase {
 public:
  ChickenHeadPlan(const drake::math::RigidTransformd &X_WL7_0,
                  double duration,
                  double control_time_step,
                  const Eigen::Ref<const Eigen::VectorXd> &nominal_joint_angles,
                  const drake::multibody::MultibodyPlant<double> *plant);
  ~ChickenHeadPlan() override;

  void Step(const State &state, double control_period, double t,
            Command *cmd) const override;
  double duration() const override {return duration_;};
 private:
  const double duration_{0};
  // Relative transform between the frame L7 and the frame tool frame T.
  const drake::math::RigidTransformd X_L7T_ = drake::math::RigidTransformd(
      drake::math::RollPitchYawd(Eigen::Vector3d(0, -M_PI / 2, 0)),
      Eigen::Vector3d(0, 0, 0.255));
  const drake::math::RigidTransformd X_WCd_;
  const drake::multibody::Frame<double>& frame_L7_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  std::unique_ptr<
  drake::manipulation::planner::DifferentialInverseKinematicsParameters>
  params_;

  // Relative pose estimator subscription.
  void ReceiveRelativePose() const;
  std::unique_ptr<drake::lcm::DrakeLcm> owned_lcm_;
  std::unique_ptr<drake::lcm::Subscriber<drake::lcmt_robot_state>> rpe_sub_;
  std::thread sub_thread_;
  std::atomic<bool> stop_flag_{false};
  mutable std::mutex mutex_rpe_;
  mutable drake::math::RigidTransformd X_TC_;
};
