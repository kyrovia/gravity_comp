#pragma once

#include <Eigen/Core>

///types.hpp写全局通用的常量
namespace gct {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.81;

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

}  // namespace gct
