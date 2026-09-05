#pragma once

#include "gravity_comp/types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace gravity_comp::mujoco {

/// Torque-controlled MuJoCo plant for a named set of 1-DoF arm joints.
class ArmSimulator {
 public:
  ArmSimulator(const std::string& scene_path,
               std::vector<std::string> joint_names);
  ~ArmSimulator();

  ArmSimulator(const ArmSimulator&) = delete;
  ArmSimulator& operator=(const ArmSimulator&) = delete;
  ArmSimulator(ArmSimulator&&) noexcept;
  ArmSimulator& operator=(ArmSimulator&&) noexcept;

  int num_joints() const { return static_cast<int>(joint_names_.size()); }
  const std::vector<std::string>& joint_names() const { return joint_names_; }
  double time_step() const;

  void reset(const VectorXd& q, const VectorXd& v);
  void set_state(const VectorXd& q, const VectorXd& v);
  void apply_torques(const VectorXd& tau);
  void step();

  VectorXd positions() const;
  VectorXd velocities() const;

  /// Returns MuJoCo qfrc_bias at zero velocity without mutating live state.
  VectorXd gravity_torque() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::string> joint_names_;
  std::vector<int> qpos_addresses_;
  std::vector<int> dof_addresses_;
  std::vector<int> actuator_ids_;
};

}  // namespace gravity_comp::mujoco
