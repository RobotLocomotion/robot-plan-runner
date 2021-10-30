#pragma once
#include <yaml-cpp/yaml.h>

#include "plans/plan_factory_base.h"

class IiwaPlanFactory : public PlanFactory {
public:
  IiwaPlanFactory(const YAML::Node &config);

  ~IiwaPlanFactory() override = default;

  [[nodiscard]] std::unique_ptr<PlanBase>
  MakePlan(const drake::lcmt_robot_plan &msg_plan) const override;

private:
  // q_xyz.size() == 7. q_xyz[0:4] are the w, x, y and z components of the
  // quaternion. q_xyz[4:7] are the x, y and z components of the translation.
  template <class T>
  static drake::math::RigidTransformd TransformFromRobotStateMsg(
      const std::vector<T>& q_xyz);
  const YAML::Node &config_;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeJointSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeTaskSpaceTrajectoryPlan(const drake::lcmt_robot_plan &msg_plan) const;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeChickenHeadPlan(const drake::lcmt_robot_plan &msg_plan) const;
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeSqueegeePlan(const drake::lcmt_robot_plan &msg_plan) const;  
  [[nodiscard]] std::unique_ptr<PlanBase>
  MakeScrewdriverPlan(const drake::lcmt_robot_plan &msg_plan) const;    
};

template <class T>
drake::math::RigidTransformd IiwaPlanFactory::TransformFromRobotStateMsg(
    const std::vector<T>& q_xyz) {
  DRAKE_THROW_UNLESS(q_xyz.size() == 7);
  const auto Q = Eigen::Quaterniond(q_xyz[0], q_xyz[1], q_xyz[2], q_xyz[3]);
  const auto p = Eigen::Vector3d(q_xyz[4], q_xyz[5], q_xyz[6]);
  return {Q, p};
}
