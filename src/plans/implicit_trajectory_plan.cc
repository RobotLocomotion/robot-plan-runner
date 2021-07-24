#include "drake_lcmtypes/drake/lcmt_robot_state.hpp"

#include "implicit_trajectory_plan.h"

using drake::manipulation::planner::DifferentialInverseKinematicsResult;
using drake::manipulation::planner::DifferentialInverseKinematicsStatus;
using drake::manipulation::planner::internal::DoDifferentialInverseKinematics;
using drake::manipulation::planner::ComputePoseDiffInCommonFrame;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::symbolic::Expression;
using drake::Vector3;
using drake::Vector6;
using drake::MatrixX;
using drake::AutoDiffXd;

using std::cout;
using std::endl;

using std::sin;
using std::cos;
using std::pow;

void ImplicitTrajectoryPlan::Step(const State &state, double control_period,
                                   double t, Command *cmd) const {

  // 1. Update diffik mbp with the current status of the robot.
  plant_->SetPositions(plant_context_.get(), state.q);

  // 2. Ask diffik to solve for desired position.
  const drake::math::RigidTransformd X_WTr(quat_traj_.orientation(t),
                                            xyz_traj_.value(t));
  const auto& frame_W = plant_->world_frame();
  const auto X_WE = plant_->CalcRelativeTransform(
      *plant_context_, frame_W, frame_E_);
  const auto X_WT = X_WE * X_ET_;

  // 3. Setup Mathematical program.
  const auto solver = drake::solvers::OsqpSolver();
  auto prog = MathematicalProgram();
  // The variables are read [wx, wy, wz, vx, vy, vz]
  auto V_TTn = prog.NewContinuousVariables(6, "V_TTn"); 
  auto wx = V_TTn[0];
  auto wy = V_TTn[1];
  auto wz = V_TTn[2];
  auto vx = V_TTn[3];
  auto vy = V_TTn[4];
  auto vz = V_TTn[5];

  // The variables are read [tx, ty, tz, fx, fy, fz].
  // These are slack variables that are decided by the constraints.
  auto F_CTn = prog.NewContinuousVariables(6, "F_CTn");

  // 4. Add cost. The cost is how much the new deformation deviates from 
  // the tool reference X_WTr. In order to end up with a quadratic cost, 
  // what we do here is evaluate the Frobenius norm of 
  // \| X_WTr - X_WT * X_TTn \|_F where X_TTn is parametrized using a 
  // local approximation of a rotation matrix of I + skew([wx, wy, wz]),
  // and a trnaslation of [vx, vy vz].
  Eigen::Matrix<Expression, 4, 4> X_TTn;
  X_TTn << 1, -wz, wy, vx, 
           wz, 1, -wx, vy,
           -wy, wx, 1, vz,
           0, 0, 0 , 1;

  // Evaluate difference between Tn and Tr in world frame. We use this
  // instead of X_TnTr to avoid bilinear terms.
  auto X_TnTr_W = X_WT.GetAsMatrix4().cast<Expression>() * (
  X_TTn) - X_WTr.GetAsMatrix4().cast<Expression>();
   /*
  auto X_TnTr = (X_WTr.GetAsMatrix4().cast<Expression>().inverse() * (
   X_WT.GetAsMatrix4().cast<Expression>() * X_TTn)).inverse();
  */

  Expression cost = 0.0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      cost += pow(X_TnTr_W(i,j), 2);
    }
  }

  /*
  // Weight position terms.
  for (int i = 0; i < 3; i++) {
    cost += 10.0 * pow(X_TnTr(i,3), 2);
  }
  */
  
  prog.AddQuadraticCost(cost);  

  // 5. Add constraints through passing everything through bushing element.
  // Use autodiff to linearize the constraint between V_TTn_AD and F_TnC.
  auto V_TTn_AD = drake::math::initializeAutoDiff(Eigen::Matrix<double, 6, 1>::Zero());
  auto wx_AD = V_TTn_AD[0];
  auto wy_AD = V_TTn_AD[1];
  auto wz_AD = V_TTn_AD[2];
  auto vx_AD = V_TTn_AD[3];
  auto vy_AD = V_TTn_AD[4];
  auto vz_AD = V_TTn_AD[5];

  // We will approximate X_TTn in this case with a roll-pitch-yaw. Under small
  // angles, we shouldn't see a big difference, and we get access to valid
  // rotation matrices.
  auto X_TTn_AD = drake::math::RigidTransform<AutoDiffXd>(
    drake::math::RollPitchYaw<AutoDiffXd>(V_TTn_AD.head(3)), V_TTn_AD.tail(3));

  // Compute new displacement in the bushing element.
  auto X_TnC_AD = (X_TC_.cast<AutoDiffXd>().inverse() * X_TTn_AD).inverse();
  Eigen::Matrix<AutoDiffXd, 6, 1> V_TnC_AD;
  auto w_TnC_AD = X_TTn_AD.rotation().inverse() * (V_TC_.head(3) - V_TTn_AD.head(3));
  auto v_TnC_AD = X_TTn_AD.rotation().inverse() * (V_TC_.tail(3) - V_TTn_AD.tail(3));
  V_TnC_AD << w_TnC_AD, v_TnC_AD;

  auto rpy_TnC_AD = drake::math::RollPitchYaw<AutoDiffXd>(X_TnC_AD.rotation());
  auto tau_TnC_AD = 
    Krpy_.asDiagonal() * rpy_TnC_AD.vector() + 
    Drpy_.asDiagonal() * V_TnC_AD.head(3);
  auto f_TnC_AD = 
    Kxyz_.asDiagonal() * X_TnC_AD.translation() + 
    Dxyz_.asDiagonal() * V_TnC_AD.tail(3);

  Eigen::Matrix<AutoDiffXd, 6, 1> F_TnC_AD;
  F_TnC_AD << tau_TnC_AD, f_TnC_AD;

  // Convert this force to force on C. Involves action-reaction and coordiante
  // transform.
  Eigen::Matrix<AutoDiffXd, 6, 1> F_CTn_AD;
  auto tau_CTn_AD = X_TnC_AD.rotation().inverse() * (-F_TnC_AD.head(3));
  auto f_CTn_AD = X_TnC_AD.rotation().inverse() * (-F_TnC_AD.tail(3));
  F_CTn_AD << tau_CTn_AD, f_CTn_AD;
  
  // Now evaluate the Jacobian and add a constraint between F_TnC and V_TTn. 
  auto J_F_CTn_V_TTn = drake::math::autoDiffToGradientMatrix(F_CTn_AD);

  // To do Taylor approximation, compute the current force F_CT.
  Eigen::Matrix<double, 6, 1> F_CT;
  auto tau_CT = X_TC_.rotation().inverse() * (-F_TC_.head(3));
  auto f_CT = X_TC_.rotation().inverse() * (-F_TC_.tail(3));
  F_CT << tau_CT, f_CT;

  // Add the linearized bushing constraint.
  prog.AddConstraint(F_CTn == J_F_CTn_V_TTn * V_TTn + F_CT);

  // 6. Add constraint between F_TnC and the desired contact forces using the 
  // force balance on the squeegee.
  auto lambdaLp_W = prog.NewContinuousVariables(3, "lambdaLp_W");
  auto lambdaCp_W = prog.NewContinuousVariables(3, "lambdaCp_W");
  auto lambdaRp_W = prog.NewContinuousVariables(3, "lambdaRp_W");

  // Convert F_CT to world frame.
  Eigen::Matrix<Expression, 6, 1> F_CTn_W;
  auto tau_CTn_W = (X_WT * X_TC_).rotation().matrix() * F_CTn.head(3);
  auto f_CTn_W = (X_WT * X_TC_).rotation().matrix() * F_CTn.tail(3);
  F_CTn_W << tau_CTn_W, f_CTn_W;

  prog.AddConstraint(F_CTn_W[1] == 0.1);
  prog.AddConstraint(F_CTn_W[5] == -0.02);

  /*

  // Write the force blanace on the lambdas on world frame.
  prog.AddConstraint(
      lambdaLp_W + lambdaCp_W + lambdaRp_W + F_CTn_W.tail(3) == 
    Eigen::Matrix<double, 3, 1>::Zero());

  // These are geometric properties of the squeegee.
  Eigen::Vector3d x_CLp(0.2, -0.1, 0.0);
  Eigen::Vector3d x_CCp(0.2, 0.0, 0.0);
  Eigen::Vector3d x_CRp(0.2, 0.1, 0.0);

  Eigen::Vector3d x_CLp_W = (X_WT * X_TC_).rotation().matrix() * x_CLp;
  Eigen::Vector3d x_CCp_W = (X_WT * X_TC_).rotation().matrix() * x_CCp;
  Eigen::Vector3d x_CRp_W = (X_WT * X_TC_).rotation().matrix() * x_CRp;

  auto tau_Lp_C = drake::math::VectorToSkewSymmetric(
    x_CLp_W.cast<Expression>()) * lambdaLp_W;
  auto tau_Cp_C = drake::math::VectorToSkewSymmetric(
    x_CCp_W.cast<Expression>()) * lambdaCp_W;    
  auto tau_Rp_C = drake::math::VectorToSkewSymmetric(
    x_CRp_W.cast<Expression>()) * lambdaRp_W;    

  prog.AddConstraint(
    tau_Lp_C + tau_Cp_C + tau_Rp_C + F_CTn_W.head(3) ==
    Eigen::Matrix<double, 3, 1>::Zero());

  // 7. Finally, add constraints on what the contact force should be.
  // We only add the z-direction constraint here because that's the one
  // we care about.
  // NOTE(terry-suh): Make sure to do force balance in W,
  prog.AddConstraint(lambdaLp_W[2] == lambda_W_[2]);
  prog.AddConstraint(lambdaCp_W[2] == lambda_W_[2]);
  prog.AddConstraint(lambdaRp_W[2] == lambda_W_[2]);
  */



  const MathematicalProgramResult deformation_result = solver.Solve(prog);

  // std::cout << deformation_result.is_success() << std::endl;
  std::cout << deformation_result.get_optimal_cost() << std::endl;
  const auto V_TTnstar = deformation_result.GetSolution(V_TTn);
  const auto F_CTnstar = deformation_result.GetSolution(F_CTn);
  const auto lambdaLp_Wstar = deformation_result.GetSolution(lambdaLp_W);
  const auto lambdaCp_Wstar = deformation_result.GetSolution(lambdaCp_W);
  const auto lambdaRp_Wstar = deformation_result.GetSolution(lambdaRp_W);

  /*
  std::cout << V_TTnstar << std::endl;
  std::cout << "=======" << std::endl;
  std::cout << F_CTnstar << std::endl;
  std::cout << "=======" << std::endl;
  std::cout << lambdaLp_Wstar << std::endl;  
  std::cout << "=======" << std::endl;  
  std::cout << lambdaCp_Wstar << std::endl;  
  std::cout << "=======" << std::endl;  
  std::cout << lambdaRp_Wstar << std::endl;  
  std::cout << "=======" << std::endl;  
  std::cout << X_WT * X_TC_ << std::endl;
  */

  auto X_TTn_star = drake::math::RigidTransform<double>(
    drake::math::RollPitchYaw<double>(V_TTnstar.head(3)), V_TTnstar.tail(3));

  auto X_WTn_star = X_WT * X_TTn_star;

    // A factor of 0.1 is multiplied because X_WT_corrected is updated from 
  // relative pose, which is published at ~20Hz. Since the robot is sending
  // q_cmd at 200Hz, we apply a zero-order hold this way.
  Vector6<double> V_WT_desired =
      0.1 * ComputePoseDiffInCommonFrame(
          X_WT.GetAsIsometry3(), X_WTn_star.GetAsIsometry3()) /
          params_->get_timestep();

  // If the velocity is too large, saturate it.
  if (V_WT_desired.norm() > 0.3) {
    V_WT_desired = 0.3 * V_WT_desired / V_WT_desired.norm();
  }


  MatrixX<double> J_WT(6, plant_->num_velocities());
  plant_->CalcJacobianSpatialVelocity(*plant_context_,
                                    drake::multibody::JacobianWrtVariable::kV,
                                    frame_E_, X_ET_.translation(),
                                    frame_W, frame_W, &J_WT);


  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      state.q, state.v, X_WT, J_WT,
      drake::multibody::SpatialVelocity<double>(V_WT_desired), *params_);

  // 3. Check for errors and integrate.
  if (result.status != DifferentialInverseKinematicsStatus::kSolutionFound) {
    // Set the command to NAN so that state machine will detect downstream and
    // go to error state.
    cmd->q_cmd = NAN * Eigen::VectorXd::Zero(7);
    // TODO(terry-suh): how do I tell the use that the state machine went to
    // error because of this precise reason? Printing the error message here
    // seems like a good start, but we'll need to handle this better.
    std::cout << "DoDifferentialKinematics Failed to find a solution."
              << std::endl;
  } else {
    cmd->q_cmd = state.q + control_period * result.joint_velocities.value();
    cmd->tau_cmd = Eigen::VectorXd::Zero(7);
  }
}

ImplicitTrajectoryPlan::~ImplicitTrajectoryPlan() {
  is_running_ = false;
  // Wait for all threads to terminate.
  for (auto &a : threads_) {
    if (a.second.joinable()) {
      a.second.join();
    }
  }
}

void ImplicitTrajectoryPlan::SubscribeForceTorque() {
  auto sub = lcm_->subscribe("FT",
    &ImplicitTrajectoryPlan::HandleForceTorqueStatus, this);
  sub->setQueueCapacity(1);
  while(true) {
    if (lcm_->handleTimeout(10) < 0) {
      break;
    }
    if (!is_running_) {
      break;
    }
  }
}

void ImplicitTrajectoryPlan::SubscribeVelocity() {
  auto sub = lcm_->subscribe("RELATIVE_VELOCITY",
    &ImplicitTrajectoryPlan::HandleVelocityStatus, this);
  sub->setQueueCapacity(1);
  while(true) {
    if (lcm_->handleTimeout(10) < 0) {
      break;
    }
    if (!is_running_) {
      break;
    }    
  }
}

void ImplicitTrajectoryPlan::SubscribePose() {
  auto sub = lcm_->subscribe("RELATIVE_POSE",
    &ImplicitTrajectoryPlan::HandlePoseStatus, this);
  sub->setQueueCapacity(1);
  while(true) {
    if (lcm_->handleTimeout(10) < 0) {
      break;
    }
    if (!is_running_) {
      break;
    }    
  }
}

void ImplicitTrajectoryPlan::HandleForceTorqueStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    F_TC_ = data_eigen.cast<double>();
}      


void ImplicitTrajectoryPlan::HandleVelocityStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    V_TC_ = data_eigen.cast<double>();    
}

void ImplicitTrajectoryPlan::HandlePoseStatus(
  const lcm::ReceiveBuffer *, const std::string &channel,
  const drake::lcmt_robot_state *status_msg) {

    const int num_vars = (*status_msg).num_joints;
    const std::vector<float> data = (*status_msg).joint_position;
    auto data_eigen = Eigen::Map<const Eigen::VectorXf>(data.data(), num_vars);
    const auto X_TC = data_eigen.cast<double>();
    const Eigen::Quaterniond q_TC(X_TC[0], X_TC[1], X_TC[2], X_TC[3]);
    X_TC_ = drake::math::RigidTransformd(q_TC, X_TC.tail(3));
}
