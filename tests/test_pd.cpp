#include "gct/two_link_toy.hpp"

#include <catch2/catch_test_macros.hpp>

using gct::VectorXd;
using gct::toy::pd_control;
using gct::toy::TwoLinkToyPlant;

namespace {

double run_fixed_target(bool use_gc) {
  TwoLinkToyPlant plant;
  VectorXd q_des(2);
  q_des << 90.0 * gct::kPi / 180.0, -45.0 * gct::kPi / 180.0;
  const VectorXd v_des = VectorXd::Zero(2);
  VectorXd kp(2);
  kp << 18.0, 13.0;
  VectorXd kd(2);
  kd << 4.8, 3.3;
  const double dt = 0.01;
  const int steps = 400;
  for (int i = 0; i < steps; ++i) {
    const VectorXd g =
        use_gc ? plant.gravity_torque() : VectorXd::Zero(2);
    const VectorXd tau =
        pd_control(q_des, v_des, plant.q, plant.v, kp, kd, g);
    plant.step(tau, dt);
  }
  return (q_des - plant.q).cwiseAbs().maxCoeff();
}

}  // namespace

TEST_CASE("PD + gravity reduces 2-link steady-state error") {
  const double err_no_gc = run_fixed_target(false);
  const double err_gc = run_fixed_target(true);
  REQUIRE(err_gc < err_no_gc);
  REQUIRE(err_gc < 0.05);
}
