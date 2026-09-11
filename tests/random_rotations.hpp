// Seeded random rotation vectors used by the Spec tests so failures reproduce.
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <random>

namespace ranger_lio::test {

class RotationSampler {
 public:
  explicit RotationSampler(unsigned seed = 42) : gen_(seed), normal_(0.0, 1.0), uni_(0.0, 1.0) {}

  Eigen::Vector3d axis() {
    Eigen::Vector3d a(normal_(gen_), normal_(gen_), normal_(gen_));
    return a.normalized();
  }
  // |theta| uniform on (lo, hi).
  Eigen::Vector3d theta(double lo = 1e-3, double hi = M_PI - 1e-3) { return axis() * (lo + (hi - lo) * uni_(gen_)); }
  Eigen::Vector3d thetaNearPi() { return theta(M_PI - 1e-4, M_PI - 1e-9); }
  Eigen::Vector3d vector(double scale = 1.0) { return scale * Eigen::Vector3d(normal_(gen_), normal_(gen_), normal_(gen_)); }

 private:
  std::mt19937 gen_;
  std::normal_distribution<double> normal_;
  std::uniform_real_distribution<double> uni_;
};

}  // namespace ranger_lio::test
