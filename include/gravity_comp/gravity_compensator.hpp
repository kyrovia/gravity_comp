#pragma once

#include "gravity_comp/pin_model.hpp"

namespace gravity_comp {

struct CompensatorConfig {
  VectorXd kp;
  VectorXd kd;
  /// Optional command limits. Empty uses limits supplied by the model.
  VectorXd torque_limits;
  /// When true, clip commanded torque to finite positive limits.
  bool clip_torque{false};
};

/// Torque-mode gravity feedforward on top of Pinocchio RNEA.
///
/// Primary law is τ = g(q). PD + g is the usual tracking form.
/// position_servo_command maps the same g(q) into a position-loop offset
/// when the actuator is not in torque mode.
class GravityCompensator {
 public:
  /// Stores an independent copy of the model and its computation workspace.
  explicit GravityCompensator(const PinModel& model,
                              CompensatorConfig config = {});

  const PinModel& model() const { return model_; }
  const CompensatorConfig& config() const { return config_; }

  VectorXd gravity_torque(const VectorXd& q) const;
  VectorXd pd_plus_gravity(const VectorXd& q, const VectorXd& v,
                           const VectorXd& q_des, const VectorXd& v_des) const;
  VectorXd position_servo_command(const VectorXd& q, const VectorXd& q_des,
                                  const VectorXd& v_des) const;
  VectorXd clip(const VectorXd& tau) const;

 private:
  PinModel model_;
  CompensatorConfig config_;
};

}  // namespace gravity_comp
