#include "gct/pin_model.hpp"
#include "gct_sim/mujoco_sim.hpp"
#include "ur5e.hpp"

#include "gct_paths.hpp"

#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using gct::kDefaultMjScenePath;
using gct::kDefaultUrdfPath;
using gct::PinModel;
using gct::ur5e_home;
using gct::ur5e_joint_names;
using gct::VectorXd;
using gct::sim::cross_validate_gravity;
using gct::sim::HoldOptions;
using gct::sim::MujocoSimulator;
using gct::sim::run_gravity_hold;

namespace {

PinModel load_ur5e() { return PinModel(std::string(kDefaultUrdfPath)); }

MujocoSimulator load_mujoco() {
  return MujocoSimulator(std::string(kDefaultMjScenePath), ur5e_joint_names());
}

}  // namespace

TEST_CASE("unknown joint name is rejected") {
  REQUIRE_THROWS_AS(
      MujocoSimulator(std::string(kDefaultMjScenePath),
                      std::vector<std::string>{"not_a_joint"}),
      std::runtime_error);
}

TEST_CASE("joint-name mismatch is rejected") {
  PinModel pin = load_ur5e();
  auto names = ur5e_joint_names();
  std::swap(names.front(), names.back());
  MujocoSimulator sim(std::string(kDefaultMjScenePath), names);
  REQUIRE_THROWS_AS(cross_validate_gravity(pin, sim, ur5e_home()),
                    std::invalid_argument);
}

TEST_CASE("Pinocchio g(q) matches MuJoCo qfrc_bias") {
  PinModel pin = load_ur5e();
  MujocoSimulator sim = load_mujoco();
  const std::vector<VectorXd> configs = {ur5e_home(), VectorXd::Zero(kUr5eDofs),
                                         (VectorXd(kUr5eDofs) << 0.5, -1.0, 1.2,
                                          -0.8, 0.3, -0.5)
                                             .finished()};
  for (const auto& q : configs) {
    const double err = cross_validate_gravity(pin, sim, q);
    REQUIRE(err < 1e-3);
  }
}

TEST_CASE("MuJoCo hold at Q_HOME under tau = g(q)") {
  PinModel pin = load_ur5e();
  MujocoSimulator sim = load_mujoco();
  HoldOptions opt;
  opt.duration = 3.0;
  const auto log = run_gravity_hold(sim, pin, ur5e_home(), opt);
  REQUIRE(log.q_error.cwiseAbs().maxCoeff() < 0.01);
}

TEST_CASE("MuJoCo hold at arbitrary configuration") {
  PinModel pin = load_ur5e();
  MujocoSimulator sim = load_mujoco();
  VectorXd q(kUr5eDofs);
  q << 0.5, -1.0, 1.2, -0.8, 0.3, -0.5;
  HoldOptions opt;
  opt.duration = 3.0;
  const auto log = run_gravity_hold(sim, pin, q, opt);
  REQUIRE(log.q_error.cwiseAbs().maxCoeff() < 0.01);
}

TEST_CASE("MuJoCo perturbation velocity decays") {
  PinModel pin = load_ur5e();
  MujocoSimulator sim = load_mujoco();
  HoldOptions opt;
  opt.duration = 5.0;
  opt.perturb_at = 0.5;
  opt.perturb_duration = 0.2;
  opt.perturb_torque.resize(kUr5eDofs);
  opt.perturb_torque << 0.0, 15.0, 10.0, 5.0, 0.0, 0.0;
  const auto log = run_gravity_hold(sim, pin, ur5e_home(), opt);
  REQUIRE(log.v.bottomRows(200).cwiseAbs().maxCoeff() < 0.5);
}
