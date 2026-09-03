#include "gct/gravity_compensator.hpp"
#include "gct/pin_model.hpp"
#include "ur5e.hpp"

#include "gct_paths.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

using gct::CompensatorConfig;
using gct::GravityCompensator;
using gct::kDefaultUrdfPath;
using gct::kUr5eDofs;
using gct::PinModel;
using gct::ur5e_home;
using gct::VectorXd;

namespace {

PinModel load_ur5e() { return PinModel(std::string(kDefaultUrdfPath)); }

}  // namespace

TEST_CASE("Pinocchio gravity is nonzero at UR5e home") {
  const PinModel pin = load_ur5e();
  const VectorXd g = pin.gravity(ur5e_home());
  REQUIRE(g.size() == kUr5eDofs);
  REQUIRE(g.norm() > 1.0);
}

TEST_CASE("gravity is RNEA at zero velocity and acceleration") {
  const PinModel pin = load_ur5e();
  const VectorXd zero = VectorXd::Zero(pin.nv());
  const std::vector<VectorXd> configs = {
      ur5e_home(), VectorXd::Zero(kUr5eDofs),
      (VectorXd(kUr5eDofs) << 0.5, -1.0, 1.2, -0.8, 0.3, -0.5).finished()};
  for (const auto& q : configs) {
    const VectorXd g = pin.gravity(q);
    const VectorXd tau = pin.rnea(q, zero, zero);
    REQUIRE((g - tau).cwiseAbs().maxCoeff() < 1e-12);
  }
}

TEST_CASE("joint names match the UR5e arm") {
  const PinModel pin = load_ur5e();
  REQUIRE(pin.joint_names() == gct::ur5e_joint_names());
}

TEST_CASE("gravity_torque equals RNEA gravity when clipping is off") {
  const PinModel pin = load_ur5e();
  GravityCompensator gc(pin);
  const VectorXd q = ur5e_home();
  REQUIRE((gc.gravity_torque(q) - pin.gravity(q)).cwiseAbs().maxCoeff() <
          1e-12);
}

TEST_CASE("clip respects URDF effort limits") {
  const PinModel pin = load_ur5e();
  CompensatorConfig cfg;
  cfg.clip_torque = true;
  GravityCompensator gc(pin, cfg);
  const VectorXd limits = pin.torque_limits();
  VectorXd huge = VectorXd::Constant(pin.nv(), 1e6);
  const VectorXd clipped = gc.clip(huge);
  REQUIRE((clipped - limits).cwiseAbs().maxCoeff() < 1e-12);
}

TEST_CASE("position servo feedforward reconstructs PD + g at rest") {
  PinModel pin = load_ur5e();
  CompensatorConfig cfg;
  cfg.kp = VectorXd::Constant(kUr5eDofs, 80.0);
  cfg.kd = VectorXd::Constant(kUr5eDofs, 8.0);
  GravityCompensator gc(pin, cfg);

  VectorXd q = ur5e_home();
  VectorXd q_des = q;
  q_des(0) += 0.05;
  q_des(3) -= 0.03;
  const VectorXd v = VectorXd::Zero(kUr5eDofs);
  const VectorXd v_des = VectorXd::Zero(kUr5eDofs);

  const VectorXd ctrl = gc.position_servo_command(q, q_des, v_des);
  const VectorXd tau_servo =
      cfg.kp.cwiseProduct(ctrl - q) - cfg.kd.cwiseProduct(v);
  const VectorXd tau_pd = gc.pd_plus_gravity(q, v, q_des, v_des);
  REQUIRE((tau_servo - tau_pd).norm() == Catch::Approx(0.0).margin(1e-9));
}
