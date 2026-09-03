#pragma once

#include "gct/types.hpp"

#include <array>
#include <string>
#include <vector>

namespace gct {

/// Lab 3 UR5e + Robotiq payload. Not part of the core library API.
constexpr int kUr5eDofs = 6;

inline constexpr std::array<const char*, kUr5eDofs> kUr5eJointNames = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
};

inline std::vector<std::string> ur5e_joint_names() {
  return {kUr5eJointNames.begin(), kUr5eJointNames.end()};
}

inline VectorXd ur5e_home() {
  VectorXd q(kUr5eDofs);
  q << -kPi / 2.0, -kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0;
  return q;
}

}  // namespace gct
