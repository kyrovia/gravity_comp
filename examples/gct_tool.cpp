#include "gct/gct.hpp"
#include "gct_sim/mujoco_sim.hpp"
#include "ur5e.hpp"

#include "gct_paths.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

gct::VectorXd parse_csv(const std::string& text, int expected) {
  gct::VectorXd v(expected);
  std::stringstream ss(text);
  std::string item;
  int i = 0;
  while (std::getline(ss, item, ',') && i < expected) {
    v(i++) = std::stod(item);
  }
  if (i != expected) {
    throw std::runtime_error("expected comma-separated joint values");
  }
  return v;
}

void usage() {
  std::cerr << "usage:\n"
            << "  gct_tool gravity [--urdf FILE] [--q q1,q2,...]\n"
            << "  gct_tool parity  [--urdf FILE] [--scene FILE] [--q ...]\n"
            << "  gct_tool hold    [--urdf FILE] [--scene FILE] [--duration S]\n";
}

}  // namespace

int main(int argc, char** argv) {
  using namespace gct;
  using namespace gct::sim;
  if (argc < 2) {
    usage();
    return 2;
  }
  const std::string cmd = argv[1];

  std::string urdf = kDefaultUrdfPath;
  std::string scene = kDefaultMjScenePath;
  std::string q_text;
  double duration = 3.0;
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--urdf" && i + 1 < argc) {
      urdf = argv[++i];
    } else if (arg == "--scene" && i + 1 < argc) {
      scene = argv[++i];
    } else if (arg == "--q" && i + 1 < argc) {
      q_text = argv[++i];
    } else if (arg == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else {
      usage();
      return 2;
    }
  }

  PinModel pin(urdf);
  GravityCompensator gc(pin);
  VectorXd q = q_text.empty() ? ur5e_home() : parse_csv(q_text, pin.nv());

  if (cmd == "gravity") {
    const VectorXd g = gc.gravity_torque(q);
    const auto names = pin.joint_names();
    for (int i = 0; i < g.size(); ++i) {
      const std::string name =
          i < static_cast<int>(names.size()) ? names[static_cast<std::size_t>(i)]
                                            : ("joint" + std::to_string(i));
      std::cout << name << "  " << g(i) << "\n";
    }
    return 0;
  }

  if (cmd == "parity") {
    MujocoSimulator sim(scene, pin.joint_names());
    const double err = cross_validate_gravity(pin, sim, q);
    std::cout << "max |g_pin - g_mj| = " << err << " Nm\n";
    return err < 1e-3 ? 0 : 1;
  }

  if (cmd == "hold") {
    MujocoSimulator sim(scene, pin.joint_names());
    HoldOptions opt;
    opt.duration = duration;
    const auto log = run_gravity_hold(sim, pin, q, opt);
    const double max_err = log.q_error.cwiseAbs().maxCoeff();
    std::cout << "max |q - q0| = " << max_err << " rad\n";
    return max_err < 0.01 ? 0 : 1;
  }

  usage();
  return 2;
}
