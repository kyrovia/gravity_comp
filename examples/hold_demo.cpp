#include "gct/gct.hpp"
#include "gct_sim/mujoco_sim.hpp"
#include "ur5e.hpp"

#include "gct_paths.hpp"

#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  using namespace gct;
  using namespace gct::sim;

  std::string urdf = kDefaultUrdfPath;
  std::string scene = kDefaultMjScenePath;
  if (argc >= 2) {
    urdf = argv[1];
  }
  if (argc >= 3) {
    scene = argv[2];
  }

  std::cout << "============================================================\n";
  std::cout << "Gravity Compensation — Pinocchio + MuJoCo (UR5e)\n";
  std::cout << "============================================================\n";
  std::cout << "URDF : " << urdf << "\n";
  std::cout << "Scene: " << scene << "\n";

  PinModel pin(urdf);
  MujocoSimulator sim(scene, ur5e_joint_names());
  GravityCompensator gc(pin);

  const VectorXd q0 = ur5e_home();
  const VectorXd g_home = gc.gravity_torque(q0);
  std::cout << "\nPinocchio g(q_home) [Nm]:\n  [";
  for (int i = 0; i < g_home.size(); ++i) {
    std::cout << std::showpos << std::fixed << std::setprecision(4) << g_home(i);
    if (i + 1 < g_home.size()) {
      std::cout << ", ";
    }
  }
  std::cout << std::noshowpos << "]\n";

  const double parity = cross_validate_gravity(pin, sim, q0);
  std::cout << "\nPinocchio vs MuJoCo max |g_pin - g_mj| at Q_HOME: "
            << parity << " Nm\n";
  std::cout << "  Criterion: < 1e-3 -> [" << (parity < 1e-3 ? "PASS" : "FAIL")
            << "]\n";

  std::cout << "\nTest 1: MuJoCo hold under tau = g(q) (5s)...\n";
  HoldOptions hold;
  hold.duration = 5.0;
  const auto r1 = run_gravity_hold(sim, pin, q0, hold);
  const double max_err = r1.q_error.cwiseAbs().maxCoeff();
  std::cout << "  Max position error: " << max_err << " rad\n";
  std::cout << "  Criterion: < 0.01 rad -> ["
            << (max_err < 0.01 ? "PASS" : "FAIL") << "]\n";

  std::cout << "\nTest 2: Perturbation at t=1.0s, then hold (5s)...\n";
  MujocoSimulator sim2(scene, ur5e_joint_names());
  HoldOptions p;
  p.duration = 5.0;
  p.perturb_at = 1.0;
  p.perturb_duration = 0.2;
  p.perturb_torque.resize(kUr5eDofs);
  p.perturb_torque << 0.0, 20.0, 10.0, 5.0, 0.0, 0.0;
  const auto r2 = run_gravity_hold(sim2, pin, q0, p);
  const double final_vel = r2.v.bottomRows(100).cwiseAbs().maxCoeff();
  std::cout << "  Final velocity (last 100 steps): " << final_vel << " rad/s\n";
  std::cout << "  Criterion: < 0.5 rad/s -> ["
            << (final_vel < 0.5 ? "PASS" : "FAIL") << "]\n";

  std::cout << "\nDone.\n";
  return (parity < 1e-3 && max_err < 0.01 && final_vel < 0.5) ? 0 : 1;
}
