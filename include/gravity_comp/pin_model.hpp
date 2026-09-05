#pragma once

#include "gravity_comp/types.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gravity_comp {

/// Uniform armature override for all joints. Omit (default) to keep values from
/// the model file: MJCF joint armature, or URDF + companion SRDF rotor params.
struct UniformArmature {
  double value;
};

class PinModel {
 public:
  explicit PinModel(const std::string& model_path,
                    std::optional<UniformArmature> armature_override = std::nullopt);

  PinModel(const PinModel& other);
  PinModel& operator=(const PinModel& other);
  PinModel(PinModel&& other) noexcept;
  PinModel& operator=(PinModel&& other) noexcept;
  ~PinModel();

  int nv() const;
  int nq() const;

  std::vector<std::string> joint_names() const;
  VectorXd torque_limits() const;
  VectorXd armature() const;

  /// Reuses internal Pinocchio workspace and is not safe for concurrent calls
  /// on the same PinModel instance.
  VectorXd gravity(const VectorXd& q) const;
  VectorXd rnea(const VectorXd& q, const VectorXd& v, const VectorXd& a) const;

  const std::string& model_path() const { return model_path_; }
  [[deprecated("use model_path()")]]
  const std::string& urdf_path() const { return model_path_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::string model_path_;
};

}  // namespace gravity_comp
