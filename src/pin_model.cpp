#include "gct/pin_model.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string>

namespace gct {
namespace {

bool ends_with_ci(const std::string& path, const char* suffix) {
  const std::size_t suffix_len = std::char_traits<char>::length(suffix);
  if (path.size() < suffix_len) {
    return false;
  }
  for (std::size_t i = 0; i < suffix_len; ++i) {
    const char a = static_cast<char>(
        std::tolower(static_cast<unsigned char>(path[path.size() - suffix_len + i])));
    const char b = static_cast<char>(
        std::tolower(static_cast<unsigned char>(suffix[i])));
    if (a != b) {
      return false;
    }
  }
  return true;
}

bool is_mjcf_path(const std::string& path) {
  return ends_with_ci(path, ".xml") || ends_with_ci(path, ".mjcf");
}

bool file_exists(const std::string& path) {
  std::ifstream in(path);
  return in.good();
}

std::string replace_extension(const std::string& path, const char* ext) {
  const std::size_t slash = path.find_last_of("/\\");
  const std::size_t dot = path.find_last_of('.');
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
    return path + ext;
  }
  return path.substr(0, dot) + ext;
}

void build_pinocchio_model(const std::string& model_path, pinocchio::Model& model) {
  if (is_mjcf_path(model_path)) {
    pinocchio::mjcf::buildModel(model_path, model);
    return;
  }

  pinocchio::urdf::buildModel(model_path, model);
  const std::string srdf_path = replace_extension(model_path, ".srdf");
  if (file_exists(srdf_path)) {
    pinocchio::srdf::loadRotorParameters(model, srdf_path);
  }
}

void apply_armature_override(pinocchio::Model& model,
                             const std::optional<UniformArmature>& override) {
  if (override.has_value()) {
    model.armature.setConstant(override->value);
  }
}

}  // namespace

struct PinModel::Impl {
  pinocchio::Model model;
  mutable pinocchio::Data data;
};

PinModel::PinModel(const std::string& model_path,
                   std::optional<UniformArmature> armature_override)
    : impl_(std::make_unique<Impl>()), model_path_(model_path) {
  build_pinocchio_model(model_path, impl_->model);
  if (impl_->model.nv == 0) {
    throw std::runtime_error("Model file produced an empty Pinocchio model: " +
                             model_path);
  }
  apply_armature_override(impl_->model, armature_override);
  impl_->data = pinocchio::Data(impl_->model);
}

PinModel::PinModel(const PinModel& other) : model_path_(other.model_path_) {
  if (other.impl_ == nullptr) {
    return;
  }
  impl_ = std::make_unique<Impl>();
  impl_->model = other.impl_->model;
  impl_->data = pinocchio::Data(impl_->model);
}

PinModel& PinModel::operator=(const PinModel& other) {
  if (this == &other) {
    return *this;
  }
  model_path_ = other.model_path_;
  if (other.impl_ == nullptr) {
    impl_.reset();
    return *this;
  }
  impl_ = std::make_unique<Impl>();
  impl_->model = other.impl_->model;
  impl_->data = pinocchio::Data(impl_->model);
  return *this;
}

PinModel::PinModel(PinModel&& other) noexcept = default;
PinModel& PinModel::operator=(PinModel&& other) noexcept = default;
PinModel::~PinModel() = default;

int PinModel::nv() const {
  return impl_ != nullptr ? static_cast<int>(impl_->model.nv) : 0;
}

int PinModel::nq() const {
  return impl_ != nullptr ? static_cast<int>(impl_->model.nq) : 0;
}

std::vector<std::string> PinModel::joint_names() const {
  if (impl_ == nullptr) {
    return {};
  }
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(impl_->model.njoints));
  for (pinocchio::JointIndex i = 1;
       i < static_cast<pinocchio::JointIndex>(impl_->model.njoints); ++i) {
    if (impl_->model.joints[i].nq() > 0) {
      names.push_back(impl_->model.names[i]);
    }
  }
  return names;
}

VectorXd PinModel::torque_limits() const {
  if (impl_ == nullptr) {
    return {};
  }
  return impl_->model.effortLimit;
}

VectorXd PinModel::armature() const {
  if (impl_ == nullptr) {
    return {};
  }
  return impl_->model.armature;
}

VectorXd PinModel::gravity(const VectorXd& q) const {
  return rnea(q, VectorXd::Zero(nv()), VectorXd::Zero(nv()));
}

VectorXd PinModel::rnea(const VectorXd& q, const VectorXd& v,
                        const VectorXd& a) const {
  if (impl_ == nullptr) {
    throw std::runtime_error("PinModel is empty");
  }
  if (q.size() != nq() || v.size() != nv() || a.size() != nv()) {
    throw std::invalid_argument("state vector has the wrong size");
  }
  return pinocchio::rnea(impl_->model, impl_->data, q, v, a);
}

}  // namespace gct
