#include "gravity_comp_mujoco/arm_simulator.hpp"

#include <mujoco/mujoco.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace gravity_comp::mujoco {
namespace {

struct MjDataDeleter {
  void operator()(mjData* data) const {
    if (data != nullptr) {
      mj_deleteData(data);
    }
  }
};

using UniqueMjData = std::unique_ptr<mjData, MjDataDeleter>;

int actuator_for_joint(const mjModel* model, int joint_id) {
  for (int actuator = 0; actuator < model->nu; ++actuator) {
    if (model->actuator_trntype[actuator] == mjTRN_JOINT &&
        model->actuator_trnid[2 * actuator] == joint_id) {
      return actuator;
    }
  }
  return -1;
}

void copy_values(const double* source, const std::vector<int>& addresses,
                 VectorXd& output) {
  for (int i = 0; i < output.size(); ++i) {
    output(i) = source[addresses[static_cast<std::size_t>(i)]];
  }
}

void write_values(double* destination, const std::vector<int>& addresses,
                  const VectorXd& input) {
  for (int i = 0; i < input.size(); ++i) {
    destination[addresses[static_cast<std::size_t>(i)]] = input(i);
  }
}

}  // namespace

struct ArmSimulator::Impl {
  mjModel* model{nullptr};
  UniqueMjData data;

  ~Impl() {
    if (model != nullptr) {
      mj_deleteModel(model);
    }
  }
};

ArmSimulator::ArmSimulator(const std::string& scene_path,
                           std::vector<std::string> joint_names)
    : impl_(std::make_unique<Impl>()),
      joint_names_(std::move(joint_names)) {
  if (joint_names_.empty()) {
    throw std::invalid_argument(
        "ArmSimulator requires at least one joint name");
  }

  char error[1024] = {0};
  const bool is_binary =
      scene_path.size() >= 4 &&
      scene_path.compare(scene_path.size() - 4, 4, ".mjb") == 0;
  if (is_binary) {
    impl_->model = mj_loadModel(scene_path.c_str(), nullptr);
  } else {
    impl_->model = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
  }
  if (impl_->model == nullptr) {
    throw std::runtime_error("failed to load MuJoCo model: " + scene_path +
                             (is_binary ? "" : (": " + std::string(error))));
  }

  impl_->data.reset(mj_makeData(impl_->model));
  if (impl_->data == nullptr) {
    throw std::runtime_error("mj_makeData failed");
  }

  qpos_addresses_.reserve(joint_names_.size());
  dof_addresses_.reserve(joint_names_.size());
  actuator_ids_.reserve(joint_names_.size());
  for (const auto& name : joint_names_) {
    const int joint_id =
        mj_name2id(impl_->model, mjOBJ_JOINT, name.c_str());
    if (joint_id < 0) {
      throw std::runtime_error("MuJoCo model has no joint named '" + name + "'");
    }
    const int joint_type = impl_->model->jnt_type[joint_id];
    if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE) {
      throw std::runtime_error("joint '" + name +
                               "' is not a 1-DoF hinge or slide joint");
    }
    const int actuator_id = actuator_for_joint(impl_->model, joint_id);
    if (actuator_id < 0) {
      throw std::runtime_error("MuJoCo model has no actuator for joint '" +
                               name + "'");
    }
    qpos_addresses_.push_back(impl_->model->jnt_qposadr[joint_id]);
    dof_addresses_.push_back(impl_->model->jnt_dofadr[joint_id]);
    actuator_ids_.push_back(actuator_id);
  }
}

ArmSimulator::~ArmSimulator() = default;
ArmSimulator::ArmSimulator(ArmSimulator&&) noexcept = default;
ArmSimulator& ArmSimulator::operator=(ArmSimulator&&) noexcept = default;

double ArmSimulator::time_step() const { return impl_->model->opt.timestep; }

void ArmSimulator::set_state(const VectorXd& q, const VectorXd& v) {
  if (q.size() != num_joints() || v.size() != num_joints()) {
    throw std::invalid_argument("state has the wrong size");
  }
  write_values(impl_->data->qpos, qpos_addresses_, q);
  write_values(impl_->data->qvel, dof_addresses_, v);
  mj_forward(impl_->model, impl_->data.get());
}

void ArmSimulator::reset(const VectorXd& q, const VectorXd& v) {
  mj_resetData(impl_->model, impl_->data.get());
  set_state(q, v);
}

void ArmSimulator::apply_torques(const VectorXd& tau) {
  if (tau.size() != num_joints()) {
    throw std::invalid_argument("tau has the wrong size");
  }
  write_values(impl_->data->ctrl, actuator_ids_, tau);
}

void ArmSimulator::step() { mj_step(impl_->model, impl_->data.get()); }

VectorXd ArmSimulator::positions() const {
  VectorXd output(num_joints());
  copy_values(impl_->data->qpos, qpos_addresses_, output);
  return output;
}

VectorXd ArmSimulator::velocities() const {
  VectorXd output(num_joints());
  copy_values(impl_->data->qvel, dof_addresses_, output);
  return output;
}

VectorXd ArmSimulator::gravity_torque() const {
  UniqueMjData scratch(mj_copyData(nullptr, impl_->model, impl_->data.get()));
  if (scratch == nullptr) {
    throw std::runtime_error("mj_copyData failed");
  }
  for (int address : dof_addresses_) {
    scratch->qvel[address] = 0.0;
  }
  mj_forward(impl_->model, scratch.get());
  VectorXd gravity(num_joints());
  copy_values(scratch->qfrc_bias, dof_addresses_, gravity);
  return gravity;
}

}  // namespace gravity_comp::mujoco
