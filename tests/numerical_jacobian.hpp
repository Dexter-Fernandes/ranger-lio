// Central-difference Jacobian helper shared by every Spec test (docs/01 onwards).
#pragma once

#include <Eigen/Core>
#include <functional>

namespace ranger_lio::test {

// J(i, j) = d f_i / d x_j at x, by central differences with step h.
inline Eigen::MatrixXd numericalJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
                                         const Eigen::VectorXd& x, double h = 1e-6) {
  const Eigen::VectorXd f0 = f(x);
  Eigen::MatrixXd J(f0.size(), x.size());
  for (Eigen::Index j = 0; j < x.size(); ++j) {
    Eigen::VectorXd xp = x, xm = x;
    xp(j) += h;
    xm(j) -= h;
    J.col(j) = (f(xp) - f(xm)) / (2.0 * h);
  }
  return J;
}

}  // namespace ranger_lio::test
