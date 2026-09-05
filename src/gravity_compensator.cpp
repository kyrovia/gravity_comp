#include "gct/gravity_compensator.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace gct {
namespace {

void fill_gains(VectorXd& gain, int nv, double fallback) {
  if (gain.size() == 0) {
    gain = VectorXd::Constant(nv, fallback);
  } else if (gain.size() != nv) {
    throw std::invalid_argument("gain vector has the wrong size");
  }
}

void require_size(const VectorXd& value, int expected, const char* name) {
  if (value.size() != expected) {
    throw std::invalid_argument(std::string(name) + " has the wrong size");
  }
}

void require_valid_limits(const VectorXd& limits, int nv) {
  require_size(limits, nv, "torque_limits");
  if (!limits.allFinite() || (limits.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "torque_limits must contain finite positive values");
  }
}

}  // namespace

GravityCompensator::GravityCompensator(const PinModel& model,
                                       CompensatorConfig config)
    : model_(model), config_(std::move(config)) {
  const int nv = model_.nv();
  fill_gains(config_.kp, nv, 100.0);
  fill_gains(config_.kd, nv, 10.0);
  if (config_.torque_limits.size() != 0) {
    require_valid_limits(config_.torque_limits, nv);
  }
  if (config_.clip_torque) {
    require_valid_limits(config_.torque_limits.size() != 0
                             ? config_.torque_limits
                             : model_.torque_limits(),
                         nv);
  }
}

VectorXd GravityCompensator::gravity_torque(const VectorXd& q) const {
  require_size(q, model_.nq(), "q");
  VectorXd tau = model_.gravity(q);
  if (config_.clip_torque) {
    tau = clip(tau);
  }
  return tau;
}

VectorXd GravityCompensator::pd_plus_gravity(const VectorXd& q,
                                             const VectorXd& v,
                                             const VectorXd& q_des,
                                             const VectorXd& v_des) const {
  if (model_.nq() != model_.nv()) {
    throw std::logic_error(
        "PD control currently requires a Euclidean configuration (nq == nv)");
  }
  require_size(q, model_.nq(), "q");
  require_size(v, model_.nv(), "v");
  require_size(q_des, model_.nq(), "q_des");
  require_size(v_des, model_.nv(), "v_des");
  VectorXd tau = config_.kp.cwiseProduct(q_des - q) +
                 config_.kd.cwiseProduct(v_des - v) + model_.gravity(q);
  if (config_.clip_torque) {
    tau = clip(tau);
  }
  return tau;
}

VectorXd GravityCompensator::position_servo_command(
    const VectorXd& q, const VectorXd& q_des, const VectorXd& v_des) const {
  if (model_.nq() != model_.nv()) {
    throw std::logic_error(
        "position servo currently requires a Euclidean configuration "
        "(nq == nv)");
  }
  require_size(q, model_.nq(), "q");
  require_size(q_des, model_.nq(), "q_des");
  require_size(v_des, model_.nv(), "v_des");
  const VectorXd g = model_.gravity(q);
  VectorXd ctrl(q_des.size());
  for (int i = 0; i < q_des.size(); ++i) {
    const double kp = config_.kp(i);
    if (kp == 0.0) {
      throw std::runtime_error("Kp must be nonzero for position-servo feedforward");
    }
    ctrl(i) = q_des(i) + g(i) / kp + config_.kd(i) * v_des(i) / kp;
  }
  return ctrl;
}

VectorXd GravityCompensator::clip(const VectorXd& tau) const {
  const VectorXd limits = config_.torque_limits.size() != 0
                              ? config_.torque_limits
                              : model_.torque_limits();
  if (tau.size() != limits.size()) {
    throw std::invalid_argument("torque vector has the wrong size");
  }
  require_valid_limits(limits, model_.nv());
  VectorXd out = tau;
  for (int i = 0; i < out.size(); ++i) {
    const double lim = limits(i);
    if (out(i) > lim) {
      out(i) = lim;
    } else if (out(i) < -lim) {
      out(i) = -lim;
    }
  }
  return out;
}

}  // namespace gct
