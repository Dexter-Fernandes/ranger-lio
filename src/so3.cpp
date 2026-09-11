#include "ranger_lio/so3.hpp"

#include <algorithm>
#include <cmath>

namespace ranger_lio {
namespace so3 {

namespace {
constexpr double kSmallAngle = 1e-6;   // below this use series expansions
constexpr double kNearPiSin = 1e-5;    // below this sin(phi), take the axis from the symmetric part
}  // namespace

Eigen::Matrix3d hat(const Eigen::Vector3d& a) {
  Eigen::Matrix3d A;
  A << 0.0, -a.z(), a.y(),
       a.z(), 0.0, -a.x(),
      -a.y(), a.x(), 0.0;
  return A;
}

Eigen::Vector3d vee(const Eigen::Matrix3d& A) {
  return {A(2, 1), A(0, 2), A(1, 0)};
}

Eigen::Matrix3d Exp(const Eigen::Vector3d& theta) {
  const double phi = theta.norm();
  const Eigen::Matrix3d K = hat(theta);
  if (phi < kSmallAngle) {
    return Eigen::Matrix3d::Identity() + K + 0.5 * K * K;
  }
  const double a = std::sin(phi) / phi;
  const double b = (1.0 - std::cos(phi)) / (phi * phi);
  return Eigen::Matrix3d::Identity() + a * K + b * K * K;
}

Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
  // cos(phi) from the trace, sin(phi) from the antisymmetric part; atan2 of the pair is
  // well conditioned everywhere, whereas acos of the trace alone loses ~sqrt(eps)/sin(phi)
  // near phi = pi (docs/01, "Logarithm").
  const Eigen::Vector3d w = vee(R - R.transpose());  // = 2 sin(phi) u, accurate to ~eps
  const double cos_phi = std::clamp(0.5 * (R.trace() - 1.0), -1.0, 1.0);
  const double sin_phi = 0.5 * w.norm();
  const double phi = std::atan2(sin_phi, cos_phi);
  if (phi < kSmallAngle) {
    return 0.5 * w;
  }
  if (sin_phi < kNearPiSin && cos_phi < 0.0) {  // sin is also small near 0; cos tells them apart
    // Within ~1e-5 rad of pi the axis from w has relative error eps / sin(phi). Use the
    // symmetric part instead: (R + R^T)/2 + I = (1 + cos phi) I + (1 - cos phi) u u^T, whose
    // largest column is parallel to u up to O(1 + cos phi) = O((pi - phi)^2).
    const Eigen::Matrix3d S = 0.5 * (R + R.transpose()) + Eigen::Matrix3d::Identity();
    int col = 0;
    S.colwise().norm().maxCoeff(&col);
    Eigen::Vector3d u = S.col(col).normalized();
    if (u.dot(w) < 0.0) u = -u;  // sin(phi) > 0 for phi < pi fixes the sign; at pi either is right
    return phi * u;
  }
  return (phi / (2.0 * sin_phi)) * w;
}

namespace {
// Coefficients of I + s1 K + s2 K^2 for Jr/Jl and their inverses.
struct Coeffs {
  double lin, quad;
};

Coeffs jacobianCoeffs(double phi) {
  if (phi < kSmallAngle) return {0.5, 1.0 / 6.0};
  const double p2 = phi * phi;
  return {(1.0 - std::cos(phi)) / p2, (phi - std::sin(phi)) / (p2 * phi)};
}

Coeffs jacobianInvCoeffs(double phi) {
  if (phi < kSmallAngle) return {0.5, 1.0 / 12.0};
  const double p2 = phi * phi;
  return {0.5, 1.0 / p2 - (1.0 + std::cos(phi)) / (2.0 * phi * std::sin(phi))};
}
}  // namespace

Eigen::Matrix3d Jr(const Eigen::Vector3d& theta) {
  const Eigen::Matrix3d K = hat(theta);
  const Coeffs c = jacobianCoeffs(theta.norm());
  return Eigen::Matrix3d::Identity() - c.lin * K + c.quad * K * K;
}

Eigen::Matrix3d Jl(const Eigen::Vector3d& theta) {
  const Eigen::Matrix3d K = hat(theta);
  const Coeffs c = jacobianCoeffs(theta.norm());
  return Eigen::Matrix3d::Identity() + c.lin * K + c.quad * K * K;
}

Eigen::Matrix3d JrInv(const Eigen::Vector3d& theta) {
  const Eigen::Matrix3d K = hat(theta);
  const Coeffs c = jacobianInvCoeffs(theta.norm());
  return Eigen::Matrix3d::Identity() + c.lin * K + c.quad * K * K;
}

Eigen::Matrix3d JlInv(const Eigen::Vector3d& theta) {
  const Eigen::Matrix3d K = hat(theta);
  const Coeffs c = jacobianInvCoeffs(theta.norm());
  return Eigen::Matrix3d::Identity() - c.lin * K + c.quad * K * K;
}

}  // namespace so3
}  // namespace ranger_lio
