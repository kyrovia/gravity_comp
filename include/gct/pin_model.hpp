#pragma once

#include "gct/types.hpp"

#include <memory>//unqiue_ptr
#include <string>
#include <vector>

namespace gct {

constexpr double kDefaultArmature = 0.1;


class PinModel {
 public:
  explicit PinModel(const std::string& urdf_path,
                    double armature = kDefaultArmature);

  PinModel(const PinModel& other);
  PinModel& operator=(const PinModel& other);
  PinModel(PinModel&& other) noexcept;
  PinModel& operator=(PinModel&& other) noexcept;
  ~PinModel();

  int nv() const;
  int nq() const;

  std::vector<std::string> joint_names() const;
  VectorXd torque_limits() const;

  //gravity是逆动力学公式的一个特殊情况，当加速度为0时，只考虑重力
  VectorXd gravity(const VectorXd& q) const;

  //rnea是完整的逆动力学公式，可以补偿重力、科里奥利力、离心力等
  VectorXd rnea(const VectorXd& q, const VectorXd& v, const VectorXd& a) const;

  const std::string& urdf_path() const { return urdf_path_; }

 private:
  /*
  Pimpl的设计模式，别名编译防火墙
  将类的实现细节放在cpp而不是hpp文件，避免编译过慢
  想象一下，如果有10个cpp都依赖这个hpp文件的某个类，那每次都需要编译
  */
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::string urdf_path_;
};

}  // namespace gct
