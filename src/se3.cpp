#include "ranger_lio/se3.hpp"

#include "ranger_lio/so3.hpp"

namespace ranger_lio {

SE3 SE3::fromQuaternionXYZW(double qx, double qy, double qz, double qw, const Eigen::Vector3d& t) {
  Eigen::Quaterniond q(qw, qx, qy, qz);  // Eigen ctor is scalar-first
  q.normalize();
  return SE3(q.toRotationMatrix(), t);
}

Eigen::Vector4d SE3::quaternionXYZW() const {
  Eigen::Quaterniond q(R);
  if (q.w() < 0.0) q.coeffs() = -q.coeffs();
  return q.coeffs();  // Eigen stores (x, y, z, w)
}

SE3 SE3::operator*(const SE3& other) const {
  return SE3(R * other.R, R * other.t + t);
}

Eigen::Vector3d SE3::operator*(const Eigen::Vector3d& p) const {
  return R * p + t;
}

SE3 SE3::inverse() const {
  const Eigen::Matrix3d Rt = R.transpose();
  return SE3(Rt, -Rt * t);
}

SE3 SE3::Exp(const Vector6d& xi) {
  const Eigen::Vector3d theta = xi.head<3>();
  const Eigen::Vector3d rho = xi.tail<3>();
  return SE3(so3::Exp(theta), so3::Jl(theta) * rho);
}

Vector6d SE3::Log() const {
  Vector6d xi;
  const Eigen::Vector3d theta = so3::Log(R);
  xi.head<3>() = theta;
  xi.tail<3>() = so3::JlInv(theta) * t;
  return xi;
}

Matrix6d SE3::Adj() const {
  Matrix6d A = Matrix6d::Zero();
  A.topLeftCorner<3, 3>() = R;
  A.bottomLeftCorner<3, 3>() = so3::hat(t) * R;
  A.bottomRightCorner<3, 3>() = R;
  return A;
}

Eigen::Matrix<double, 3, 6> SE3::actionJacobian(const Eigen::Vector3d& p) const {
  Eigen::Matrix<double, 3, 6> J;
  J.leftCols<3>() = -R * so3::hat(p);
  J.rightCols<3>() = R;
  return J;
}

Eigen::Matrix4d SE3::matrix() const {
  Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
  M.topLeftCorner<3, 3>() = R;
  M.topRightCorner<3, 1>() = t;
  return M;
}

}  // namespace ranger_lio
