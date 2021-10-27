#pragma once

#include <optional>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include <Eigen/Dense>

struct State {
  State() = default;
  State(const Eigen::Ref<const Eigen::VectorXd> &q,
        const Eigen::Ref<const Eigen::VectorXd> &v,
        const Eigen::Ref<const Eigen::VectorXd> &tau_ext)
      : q(q), v(v), tau_ext(tau_ext){};
  Eigen::VectorXd q;
  Eigen::VectorXd v;
  Eigen::VectorXd tau_ext;
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
  // If a child Plan needs an MBP of the robot, it needs to get it from the
  // Factory which constructs the Plan.
  drake::multibody::MultibodyPlant<double> const *const plant_{nullptr};
};
