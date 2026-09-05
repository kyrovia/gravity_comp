#include "gct/gravity_compensator.hpp"

#include <stdexcept>

namespace gct {
namespace{
    void fill_gains(VectorXd& gain, int nv, double fallback) {
        if (gain.size() == 0) {
          gain = VectorXd::Constant(nv, fallback);
        } else if (gain.size() != nv) {
          throw std::invalid_argument("gain vector has the wrong size");
        }
      }
}

GravityCompensator::GravityCompensator(const PinModel& model,
    CompensatorConfig config)
: model_(model), config_(std::move(config)) {
const int nv = model_.nv();
fill_gains(config_.kp, nv, 100.0);
fill_gains(config_.kd, nv, 10.0);
}

VectorXd GravityCompensator::gravity_torque(const VectorXd& q) const {
    VectorXd tau = model_.gravity(q);  // RNEA(q, 0, 0)
    if (config_.clip_torque) {
      tau = clip(tau);
    }
    return tau;
  }

  VectorXd GravityCompensator::pd_plus_gravity(const VectorXd& q,
    const VectorXd& v,
    const VectorXd& q_des,
    const VectorXd& v_des) const {
VectorXd tau = config_.kp.cwiseProduct(q_des - q) +
config_.kd.cwiseProduct(v_des - v) + model_.gravity(q);
if (config_.clip_torque) {
tau = clip(tau);
}
return tau;
}

VectorXd GravityCompensator::position_servo_command(
    const VectorXd& q, const VectorXd& q_des, const VectorXd& v_des) const {
  const VectorXd g = model_.gravity(q);
  //ctrl位置指令
  VectorXd ctrl(q_des.size());
  for (int i = 0; i < q_des.size(); ++i) {
    const double kp = config_.kp(i);
    if (kp == 0.0) {
      throw std::runtime_error("Kp must be nonzero for position-servo feedforward");
    }
    //tau = kp * (q_des - q) tau = kd * v_des
    ctrl(i) = q_des(i) + g(i) / kp + config_.kd(i) * v_des(i) / kp;
  }
  return ctrl;
}

}