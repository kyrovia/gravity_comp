#include "gct/gravity_compensator.hpp"
#include "gct/pin_model.hpp"
#include "gct_mujoco/arm_simulator.hpp"
#include "gct_mujoco/gravity_validation.hpp"
#include "ur5e_fixture.hpp"

#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using gct::GravityCompensator;
using gct::PinModel;
using gct::VectorXd;
using gct::test::kUr5eDofs;
using gct::test::ur5e_home;
using gct::test::ur5e_joint_names;
using gct::mujoco::ArmSimulator;
using gct::mujoco::compare_gravity;
using gct::mujoco::GravityHoldOptions;
using gct::mujoco::simulate_gravity_hold;

namespace {

PinModel load_ur5e() { return PinModel(GCT_TEST_MODEL_PATH); }

ArmSimulator load_mujoco() {
  return ArmSimulator(GCT_TEST_MUJOCO_SCENE_PATH, ur5e_joint_names());
}

}  // namespace

TEST_CASE("unknown joint name is rejected") {
  REQUIRE_THROWS_AS(
      ArmSimulator(GCT_TEST_MUJOCO_SCENE_PATH,
                   std::vector<std::string>{"not_a_joint"}),
      std::runtime_error);
}

TEST_CASE("joint-name mismatch is rejected") {
  PinModel pin = load_ur5e();
  auto names = ur5e_joint_names();
  std::swap(names.front(), names.back());
  ArmSimulator simulator(GCT_TEST_MUJOCO_SCENE_PATH, names);
  REQUIRE_THROWS_AS(compare_gravity(pin, simulator, ur5e_home()),
                    std::invalid_argument);
}

TEST_CASE("Pinocchio g(q) matches MuJoCo qfrc_bias") {
  PinModel pin = load_ur5e();
  ArmSimulator simulator = load_mujoco();
  const std::vector<VectorXd> configs = {ur5e_home(), VectorXd::Zero(kUr5eDofs),
                                         (VectorXd(kUr5eDofs) << 0.5, -1.0, 1.2,
                                          -0.8, 0.3, -0.5)
                                             .finished()};
  for (const auto& q : configs) {
    const double err = compare_gravity(pin, simulator, q);
    REQUIRE(err < 1e-3);
  }
}

TEST_CASE("MuJoCo hold at Q_HOME under tau = g(q)") {
  PinModel pin = load_ur5e();
  GravityCompensator gc(pin);
  ArmSimulator simulator = load_mujoco();
  GravityHoldOptions options;
  options.duration = 3.0;
  const auto result =
      simulate_gravity_hold(simulator, gc, ur5e_home(), options);
  REQUIRE(result.position_error.cwiseAbs().maxCoeff() < 0.01);
}

TEST_CASE("MuJoCo hold at arbitrary configuration") {
  PinModel pin = load_ur5e();
  GravityCompensator gc(pin);
  ArmSimulator simulator = load_mujoco();
  VectorXd q(kUr5eDofs);
  q << 0.5, -1.0, 1.2, -0.8, 0.3, -0.5;
  GravityHoldOptions options;
  options.duration = 3.0;
  const auto result = simulate_gravity_hold(simulator, gc, q, options);
  REQUIRE(result.position_error.cwiseAbs().maxCoeff() < 0.01);
}

TEST_CASE("MuJoCo perturbation velocity decays") {
  PinModel pin = load_ur5e();
  GravityCompensator gc(pin);
  ArmSimulator simulator = load_mujoco();
  GravityHoldOptions options;
  options.duration = 5.0;
  options.perturbation_start = 0.5;
  options.perturbation_duration = 0.2;
  options.perturbation_torque.resize(kUr5eDofs);
  options.perturbation_torque << 0.0, 15.0, 10.0, 5.0, 0.0, 0.0;
  const auto result =
      simulate_gravity_hold(simulator, gc, ur5e_home(), options);
  REQUIRE(result.velocities.bottomRows(200).cwiseAbs().maxCoeff() < 0.5);
}
