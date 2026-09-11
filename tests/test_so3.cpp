// Tests for docs/01-so3-se3.md, Spec invariants 1 to 9.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "numerical_jacobian.hpp"
#include "random_rotations.hpp"
#include "ranger_lio/so3.hpp"

using namespace ranger_lio;
using ranger_lio::test::numericalJacobian;
using ranger_lio::test::RotationSampler;

namespace {
constexpr int kSamples = 100;
const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

bool isRotation(const Eigen::Matrix3d& R, double tol) {
  return (R.transpose() * R - I3).norm() < tol && std::abs(R.determinant() - 1.0) < tol;
}
}  // namespace

TEST(SO3, HatVee) {  // invariant 1
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const Eigen::Vector3d a = s.vector(), b = s.vector();
    EXPECT_LT((so3::vee(so3::hat(a)) - a).norm(), 1e-15);
    EXPECT_LT((so3::hat(a) * b - a.cross(b)).norm(), 1e-15);
    EXPECT_LT((so3::hat(a) + so3::hat(a).transpose()).norm(), 1e-15);
  }
}

TEST(SO3, ExpLogRoundTrip) {  // invariants 2, 4
  RotationSampler s;
  EXPECT_LT(so3::Log(I3).norm(), 1e-15);
  for (int i = 0; i < kSamples; ++i) {
    const Eigen::Matrix3d R = so3::Exp(s.theta());
    EXPECT_TRUE(isRotation(R, 1e-12));
    EXPECT_LT((so3::Exp(so3::Log(R)) - R).norm(), 1e-9);
  }
  for (int i = 0; i < 20; ++i) {
    const Eigen::Matrix3d R = so3::Exp(s.thetaNearPi());
    EXPECT_LT((so3::Exp(so3::Log(R)) - R).norm(), 1e-9) << "near-pi case " << i;
  }
  // Exactly pi about a coordinate axis and about a diagonal axis.
  for (const Eigen::Vector3d& u : {Eigen::Vector3d::UnitX().eval(), Eigen::Vector3d::UnitZ().eval(),
                                   Eigen::Vector3d(1, -1, 0).normalized().eval()}) {
    const Eigen::Matrix3d R = so3::Exp(M_PI * u);
    EXPECT_LT((so3::Exp(so3::Log(R)) - R).norm(), 1e-9);
    EXPECT_NEAR(so3::Log(R).norm(), M_PI, 1e-9);
  }
}

TEST(SO3, LogExpRoundTrip) {  // invariant 3
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const Eigen::Vector3d th = s.theta();
    EXPECT_LT((so3::Log(so3::Exp(th)) - th).norm(), 1e-9);
  }
  for (double mag : {1e-9, 1e-7, 1e-5}) {
    const Eigen::Vector3d th = mag * s.axis();
    EXPECT_LT((so3::Log(so3::Exp(th)) - th).norm(), 1e-9 * std::max(1.0, mag)) << "small angle " << mag;
  }
}

TEST(SO3, JacobianRelations) {  // invariants 5, 6
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const Eigen::Vector3d th = s.theta();
    EXPECT_LT((so3::Jr(th) - so3::Jl(-th)).norm(), 1e-9);
    EXPECT_LT((so3::Jr(th) - so3::Jl(th).transpose()).norm(), 1e-9);
    EXPECT_LT((so3::Jl(th) - so3::Exp(th) * so3::Jr(th)).norm(), 1e-9);
    EXPECT_LT((so3::Jr(th) * so3::JrInv(th) - I3).norm(), 1e-9);
    EXPECT_LT((so3::Jl(th) * so3::JlInv(th) - I3).norm(), 1e-9);
  }
  for (double mag : {1e-9, 1e-7, 1e-5}) {
    const Eigen::Vector3d th = mag * s.axis();
    EXPECT_LT((so3::Jr(th) * so3::JrInv(th) - I3).norm(), 1e-9);
    EXPECT_LT((so3::Jl(th) * so3::JlInv(th) - I3).norm(), 1e-9);
  }
}

TEST(SO3, JrMatchesNumerical) {  // invariant 7
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const Eigen::Vector3d th = s.theta(1e-3, 3.0);
    const Eigen::Matrix3d R = so3::Exp(th);
    auto f = [&](const Eigen::VectorXd& d) -> Eigen::VectorXd {
      return so3::Log(R.transpose() * so3::Exp(th + Eigen::Vector3d(d)));
    };
    const Eigen::MatrixXd Jnum = numericalJacobian(f, Eigen::VectorXd::Zero(3));
    EXPECT_LT((Jnum - so3::Jr(th)).norm(), 1e-6) << "sample " << i;
  }
}

TEST(SO3, JrInvMatchesNumerical) {  // invariant 8
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const Eigen::Vector3d th = s.theta(1e-3, 3.0);
    const Eigen::Matrix3d R = so3::Exp(th);
    auto f = [&](const Eigen::VectorXd& d) -> Eigen::VectorXd {
      return so3::Log(R * so3::Exp(Eigen::Vector3d(d)));
    };
    const Eigen::MatrixXd Jnum = numericalJacobian(f, Eigen::VectorXd::Zero(3));
    EXPECT_LT((Jnum - so3::JrInv(th)).norm(), 1e-6) << "sample " << i;
  }
}

TEST(SO3, Adjoint) {  // invariant 9
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const Eigen::Matrix3d R = so3::Exp(s.theta());
    const Eigen::Vector3d d = s.vector(0.5);
    EXPECT_LT((R * so3::Exp(d) * R.transpose() - so3::Exp(R * d)).norm(), 1e-9);
  }
}
