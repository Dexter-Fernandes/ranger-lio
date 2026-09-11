// SE(3) pose. Derived and specified in docs/01-so3-se3.md; conventions in docs/00-notation.md.
//   T_A_B maps points from frame B to frame A: p_A = T_A_B * p_B.
//   Tangent vectors are xi = (theta, rho): rotation first, translation second.
//   Perturbations are right (body-frame) perturbations: T * SE3::Exp(xi).
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ranger_lio {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct SE3 {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  SE3() = default;
  SE3(const Eigen::Matrix3d& R_, const Eigen::Vector3d& t_) : R(R_), t(t_) {}

  // The only place quaternion component order is decided: (x, y, z, w) as in ROS, TUM and the
  // Hilti calibration file. Eigen's constructor takes (w, x, y, z); do not call it elsewhere.
  static SE3 fromQuaternionXYZW(double qx, double qy, double qz, double qw, const Eigen::Vector3d& t);
  Eigen::Vector4d quaternionXYZW() const;  // (x, y, z, w), w >= 0

  SE3 operator*(const SE3& other) const;                       // T_A_C = T_A_B * T_B_C
  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const;   // R p + t
  SE3 inverse() const;                                         // (R^T, -R^T t)

  static SE3 Exp(const Vector6d& xi);                          // (Exp(theta), Jl(theta) rho)
  Vector6d Log() const;                                        // (Log(R), JlInv(theta) t)
  Matrix6d Adj() const;                                        // T Exp(xi) = Exp(Adj xi) T
  Eigen::Matrix<double, 3, 6> actionJacobian(const Eigen::Vector3d& p) const;  // d(T p)/d xi
  Eigen::Matrix4d matrix() const;

  const Eigen::Vector3d& translation() const { return t; }
  const Eigen::Matrix3d& rotation() const { return R; }
};

}  // namespace ranger_lio
