#pragma once

#include <optional>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include <Eigen/Dense>

// Description of a single contact.
struct ContactInfo {
  Eigen::Vector3d contact_force;                 // in world frame.
  Eigen::Vector3d contact_point;                 // in world frame.
  drake::multibody::BodyIndex contact_link_idx;  // tied to a specific MBP.
  std::optional<Eigen::Vector3d> contact_normal; // in world frame?
};

struct State {
  State() = default;
  State(const Eigen::Ref<const Eigen::VectorXd> &q,
        const Eigen::Ref<const Eigen::VectorXd> &v,
        const Eigen::Ref<const Eigen::VectorXd> &tau_ext)
      : q(q), v(v), tau_ext(tau_ext){};
  Eigen::VectorXd q;
  Eigen::VectorXd v;
  Eigen::VectorXd tau_ext;
  std::optional<std::vector<ContactInfo>> contact_results;
};

struct Command {
  Command() = default;
  Command(const Eigen::Ref<const Eigen::VectorXd> &q_cmd,
          const Eigen::Ref<const Eigen::VectorXd> &tau_cmd)
      : q_cmd(q_cmd), tau_cmd(tau_cmd){};
  Eigen::VectorXd q_cmd;
  Eigen::VectorXd tau_cmd;
};

class PlanBase {
public:
  explicit PlanBase(const drake::multibody::MultibodyPlant<double> *plant)
      : plant_(plant){};
  virtual ~PlanBase() = default;
  virtual void Step(const State &state, double control_period, double t,
                    Command *cmd) const = 0;
  [[nodiscard]] virtual double duration() const = 0;

protected:
  drake::multibody::MultibodyPlant<double> const *const plant_{nullptr};
};
