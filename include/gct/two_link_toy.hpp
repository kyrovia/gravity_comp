#pragma once

#include "gct/types.hpp"

#include <algorithm>//std::max/min
#include <cmath>//std::sin

namespace gct::toy {

class TwoLinkToyPlant {
 public:
  TwoLinkToyPlant()
      : q(VectorXd::Zero(2)),
        v(VectorXd::Zero(2)),
        inertia((VectorXd(2) << 0.22, 0.12).finished()),
        damping((VectorXd(2) << 0.55, 0.30).finished()) {}

  VectorXd q;
  VectorXd v;
  VectorXd inertia;//M简化为inertia
  VectorXd damping;//F（摩擦）简化为damping

  //计算重力产生的力矩
  VectorXd gravity_torque() const {
    VectorXd g(2);
    g(0) = 1.6 * std::sin(q(0)) + 0.18 * std::sin(q(0) + q(1));
    g(1) = 0.75 * std::sin(q(0) + q(1));
    return g;
  }

  //模拟仿真，根据输入的力矩去更新加速度和速度
  void step(const VectorXd& tau, double dt) {
    const VectorXd g = gravity_torque();
    VectorXd acc(2);
    acc(0) = (tau(0) - damping(0) * v(0) - g(0)) / inertia(0);
    acc(1) = (tau(1) - damping(1) * v(1) - g(1)) / inertia(1);
    v += acc * dt;
    q += v * dt;
  }
};

//inline不需要来回调用，适合需要高频调用，函数体不大的函数，性能更强
//PD+关节前馈，性能优于PID
inline VectorXd pd_control(const VectorXd& q_des, const VectorXd& v_des,
                           const VectorXd& q, const VectorXd& v,
                           const VectorXd& kp, const VectorXd& kd,
                           const VectorXd& gravity_term,
                           double torque_limit = 5.0) {
  VectorXd tau = kp.cwiseProduct(q_des - q) + kd.cwiseProduct(v_des - v) +
                 gravity_term;
  for (int i = 0; i < tau.size(); ++i) {
    tau(i) = std::max(-torque_limit, std::min(torque_limit, tau(i)));
  }
  return tau;
}

}  
