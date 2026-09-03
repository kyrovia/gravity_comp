#include "gct/two_link_toy.hpp"

#include <iostream>

int main() {
  using gct::kPi;
  using gct::VectorXd;
  using gct::toy::pd_control;
  using gct::toy::TwoLinkToyPlant;

  std::cout << "Lab 1 style PD vs PD + gravity on the 2-link toy plant\n";

  auto run = [](bool use_gc) {
    TwoLinkToyPlant plant;
    VectorXd q_des(2);
    q_des << 90.0 * kPi / 180.0, -45.0 * kPi / 180.0;
    const VectorXd v_des = VectorXd::Zero(2);
    VectorXd kp(2);
    kp << 18.0, 13.0;
    VectorXd kd(2);
    kd << 4.8, 3.3;
    const double dt = 0.01;
    for (int i = 0; i < 400; ++i) {
      const VectorXd g = use_gc ? plant.gravity_torque() : VectorXd::Zero(2);
      plant.step(pd_control(q_des, v_des, plant.q, plant.v, kp, kd, g), dt);
    }
    return (q_des - plant.q).cwiseAbs().maxCoeff();
  };

  const double err_no = run(false);
  const double err_gc = run(true);
  std::cout << "  steady-state |e|_inf without GC: " << err_no << " rad\n";
  std::cout << "  steady-state |e|_inf with GC:    " << err_gc << " rad\n";
  const bool pass = err_gc < err_no && err_gc < 0.05;
  std::cout << "  [" << (pass ? "PASS" : "FAIL") << "]\n";
  return pass ? 0 : 1;
}
