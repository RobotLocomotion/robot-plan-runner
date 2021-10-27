#include "plans/plan_base.h"
#include "drake/lcm/drake_lcm.h"

#include <Eigen/Dense>

class ChickenHeadPlan : public PlanBase {
 public:
  ChickenHeadPlan(drake::math::RigidTransformd  X_WT0,
                  const drake::multibody::MultibodyPlant<double> *plant);

 private:
  const drake::math::RigidTransformd X_WT0_;
  const drake::multibody::Frame<double>& frame_L7_;
  // Relative transform between the frame L7 and the frame tool frame T.
  const drake::math::RigidTransformd X_L7T_ = drake::math::RigidTransformd(
      drake::math::RollPitchYawd(Eigen::Vector3d(0, -M_PI / 2, 0)),
      Eigen::Vector3d(0, 0, 0.255));
  std::unique_ptr<drake::lcm::DrakeLcm> lcm_;
};