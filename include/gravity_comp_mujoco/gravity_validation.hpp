#pragma once

#include "gravity_comp/gravity_compensator.hpp"
#include "gravity_comp/pin_model.hpp"
#include "gravity_comp/types.hpp"
#include "gravity_comp_mujoco/arm_simulator.hpp"

#include <vector>

namespace gravity_comp::mujoco {

struct GravityHoldResult {
  std::vector<double> time;
  MatrixXd positions;
  MatrixXd velocities;
  MatrixXd torques;
  MatrixXd position_error;
};

struct GravityHoldOptions {
  double duration{5.0};
  double perturbation_start{-1.0};
  double perturbation_duration{0.2};
  VectorXd perturbation_torque;
};

/// Returns max |g_pinocchio - g_mujoco| at the supplied configuration.
double compare_gravity(const PinModel& pinocchio_model, ArmSimulator& simulator,
                       const VectorXd& q);

/// Simulates torque-mode gravity compensation from an initial configuration.
GravityHoldResult simulate_gravity_hold(
    ArmSimulator& simulator, const GravityCompensator& compensator,
    const VectorXd& initial_q, const GravityHoldOptions& options = {});

}  // namespace gravity_comp::mujoco
