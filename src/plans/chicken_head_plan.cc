#include "plans/chicken_head_plan.h"

#include <utility>

ChickenHeadPlan::ChickenHeadPlan(
    drake::math::RigidTransformd X_WT0,
    const drake::multibody::MultibodyPlant<double> *plant)
    : PlanBase(plant), X_WT0_(std::move(X_WT0)),
      frame_L7_(plant->GetFrameByName("iiwa_link_7")) {



}
