#include "gct_mujoco/gravity_validation.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace gct::mujoco {
namespace {

std::string join_names(const std::vector<std::string>& names) {
  std::ostringstream output;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      output << ", ";
    }
    output << names[i];
  }
  return output.str();
}

void require_matching_joints(const PinModel& pinocchio_model,
                             const ArmSimulator& simulator) {
  const auto pinocchio_names = pinocchio_model.joint_names();
  const auto& mujoco_names = simulator.joint_names();
  if (pinocchio_model.nv() != simulator.num_joints() ||
      pinocchio_names != mujoco_names) {
    throw std::invalid_argument(
        "Pinocchio joints [" + join_names(pinocchio_names) +
        "] != MuJoCo joints [" + join_names(mujoco_names) + "]");
  }
}

}  // namespace

double compare_gravity(const PinModel& pinocchio_model, ArmSimulator& simulator,
                       const VectorXd& q) {
  require_matching_joints(pinocchio_model, simulator);
  if (q.size() != pinocchio_model.nq()) {
    throw std::invalid_argument("q has the wrong size");
  }
  simulator.set_state(q, VectorXd::Zero(pinocchio_model.nv()));
  const VectorXd pinocchio_gravity = pinocchio_model.gravity(q);
  const VectorXd mujoco_gravity = simulator.gravity_torque();
  return (pinocchio_gravity - mujoco_gravity).cwiseAbs().maxCoeff();
}

GravityHoldResult simulate_gravity_hold(
    ArmSimulator& simulator, const GravityCompensator& compensator,
    const VectorXd& initial_q, const GravityHoldOptions& options) {
  const PinModel& pinocchio_model = compensator.model();
  require_matching_joints(pinocchio_model, simulator);
  if (pinocchio_model.nq() != pinocchio_model.nv()) {
    throw std::invalid_argument(
        "named-joint MuJoCo hold requires nq == nv");
  }
  if (initial_q.size() != pinocchio_model.nq()) {
    throw std::invalid_argument("initial_q has the wrong size");
  }

  simulator.reset(initial_q, VectorXd::Zero(pinocchio_model.nv()));
  const int step_count =
      static_cast<int>(std::lround(options.duration / simulator.time_step()));

  GravityHoldResult result;
  result.time.resize(static_cast<std::size_t>(step_count));
  result.positions.resize(step_count, pinocchio_model.nq());
  result.velocities.resize(step_count, pinocchio_model.nv());
  result.torques.resize(step_count, pinocchio_model.nv());
  result.position_error.resize(step_count, pinocchio_model.nq());

  VectorXd perturbation = options.perturbation_torque;
  if (perturbation.size() == 0) {
    perturbation = VectorXd::Zero(pinocchio_model.nv());
  } else if (perturbation.size() != pinocchio_model.nv()) {
    throw std::invalid_argument("perturbation_torque has the wrong size");
  }

  for (int step = 0; step < step_count; ++step) {
    const double time = static_cast<double>(step) * simulator.time_step();
    result.time[static_cast<std::size_t>(step)] = time;
    const VectorXd q = simulator.positions();
    result.positions.row(step) = q;
    result.velocities.row(step) = simulator.velocities();
    result.position_error.row(step) = q - initial_q;

    VectorXd torque = compensator.gravity_torque(q);
    if (options.perturbation_start >= 0.0 &&
        time >= options.perturbation_start &&
        time < options.perturbation_start + options.perturbation_duration) {
      torque += perturbation;
    }
    result.torques.row(step) = torque;
    simulator.apply_torques(torque);
    simulator.step();
  }
  return result;
}

}  // namespace gct::mujoco
