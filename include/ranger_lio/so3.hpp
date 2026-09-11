// SO(3) primitives. Derived and specified in docs/01-so3-se3.md.
#pragma once

#include <Eigen/Core>

namespace ranger_lio {
namespace so3 {

// Skew-symmetric matrix: hat(a) * b == a.cross(b).
Eigen::Matrix3d hat(const Eigen::Vector3d& a);
// Inverse of hat; A must be skew-symmetric.
Eigen::Vector3d vee(const Eigen::Matrix3d& A);

// Rodrigues: rotation by |theta| about theta/|theta|.
Eigen::Matrix3d Exp(const Eigen::Vector3d& theta);
// Inverse of Exp; returns |theta| <= pi. Handles the small-angle and near-pi limits.
Eigen::Vector3d Log(const Eigen::Matrix3d& R);

// Right and left Jacobians and their inverses:
//   Exp(theta + d) ~= Exp(theta) Exp(Jr(theta) d) ~= Exp(Jl(theta) d) Exp(theta)
//   Log(Exp(theta) Exp(d)) ~= theta + JrInv(theta) d
Eigen::Matrix3d Jr(const Eigen::Vector3d& theta);
Eigen::Matrix3d Jl(const Eigen::Vector3d& theta);
Eigen::Matrix3d JrInv(const Eigen::Vector3d& theta);
Eigen::Matrix3d JlInv(const Eigen::Vector3d& theta);

}  // namespace so3
}  // namespace ranger_lio
