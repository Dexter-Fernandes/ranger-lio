// Tests for docs/01-so3-se3.md, Spec invariants 10 to 15.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "numerical_jacobian.hpp"
#include "random_rotations.hpp"
#include "ranger_lio/se3.hpp"
#include "ranger_lio/so3.hpp"

using namespace ranger_lio;
using ranger_lio::test::numericalJacobian;
using ranger_lio::test::RotationSampler;

namespace {
constexpr int kSamples = 100;

SE3 randomPose(RotationSampler& s) { return SE3(so3::Exp(s.theta()), s.vector(5.0)); }

Vector6d randomXi(RotationSampler& s, double rotHi = M_PI - 1e-3) {
  Vector6d xi;
  xi.head<3>() = s.theta(1e-3, rotHi);
  xi.tail<3>() = s.vector(2.0);
  return xi;
}

double poseDistance(const SE3& a, const SE3& b) { return (a.matrix() - b.matrix()).norm(); }
}  // namespace

TEST(SE3, ComposeInverseAction) {  // invariant 10
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const SE3 T1 = randomPose(s), T2 = randomPose(s);
    const Eigen::Vector3d p = s.vector(3.0);
    EXPECT_LT(poseDistance(T1 * T1.inverse(), SE3()), 1e-9);
    EXPECT_LT(poseDistance(T1.inverse() * T1, SE3()), 1e-9);
    EXPECT_LT(((T1 * T2) * p - T1 * (T2 * p)).norm(), 1e-9);
    EXPECT_LT((T1.inverse() * (T1 * p) - p).norm(), 1e-9);
  }
}

TEST(SE3, ExpLogRoundTrip) {  // invariant 11
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const SE3 T = randomPose(s);
    EXPECT_LT(poseDistance(SE3::Exp(T.Log()), T), 1e-9);
    const Vector6d xi = randomXi(s);
    EXPECT_LT((SE3::Exp(xi).Log() - xi).norm(), 1e-9);
  }
  for (double mag : {1e-9, 1e-7, 1e-5}) {
    Vector6d xi;
    xi.head<3>() = mag * s.axis();
    xi.tail<3>() = s.vector(1.0);
    EXPECT_LT((SE3::Exp(xi).Log() - xi).norm(), 1e-9) << "small angle " << mag;
  }
}

TEST(SE3, Adjoint) {  // invariant 12
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const SE3 T = randomPose(s);
    const Vector6d xi = randomXi(s, 1.0);
    EXPECT_LT(poseDistance(T * SE3::Exp(xi), SE3::Exp(T.Adj() * xi) * T), 1e-9);
  }
}

TEST(SE3, ActionJacobianMatchesNumerical) {  // invariant 13
  RotationSampler s;
  for (int i = 0; i < kSamples; ++i) {
    const SE3 T = randomPose(s);
    const Eigen::Vector3d p = s.vector(3.0);
    auto f = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd { return (T * SE3::Exp(Vector6d(x))) * p; };
    const Eigen::MatrixXd Jnum = numericalJacobian(f, Eigen::VectorXd::Zero(6));
    EXPECT_LT((Jnum - T.actionJacobian(p)).norm(), 1e-6) << "sample " << i;
  }
}

TEST(SE3, HiltiExtrinsicConvention) {  // invariant 14
  // configs/hilti22.yaml, T_I_L, quaternion (x,y,z,w). See docs/00-notation.md worked example.
  const SE3 T_I_L = SE3::fromQuaternionXYZW(0.7071068, -0.7071068, 0.0, 0.0, {-0.001, -0.00855, 0.055});
  EXPECT_LT((T_I_L.R * Eigen::Vector3d::UnitZ() - Eigen::Vector3d(0, 0, -1)).norm(), 1e-6);
  EXPECT_LT((T_I_L.R * Eigen::Vector3d::UnitX() - Eigen::Vector3d(0, -1, 0)).norm(), 1e-6);
  EXPECT_LT((T_I_L.R * Eigen::Vector3d::UnitY() - Eigen::Vector3d(-1, 0, 0)).norm(), 1e-6);
  // Misreading as (w,x,y,z) would send z_L to +y_I: make sure that is not what we get.
  EXPECT_GT((T_I_L.R * Eigen::Vector3d::UnitZ() - Eigen::Vector3d(0, 1, 0)).norm(), 1.0);
  // Quaternion round trip up to sign.
  const Eigen::Vector4d q = T_I_L.quaternionXYZW();
  const SE3 back = SE3::fromQuaternionXYZW(q(0), q(1), q(2), q(3), T_I_L.t);
  EXPECT_LT(poseDistance(back, T_I_L), 1e-9);
  EXPECT_GE(q(3), 0.0);
}

TEST(SE3, GroundTruthComposition) {  // invariant 15, worked example numbers from docs/01
  const SE3 T_I_L = SE3::fromQuaternionXYZW(0.7071068, -0.7071068, 0.0, 0.0, {-0.001, -0.00855, 0.055});
  const SE3 T_W_I_a(Eigen::Matrix3d::Identity(), {10.0, -2.0, 0.5});
  const SE3 T_W_L_a = T_W_I_a * T_I_L;
  EXPECT_LT((T_W_L_a.t - Eigen::Vector3d(9.999, -2.00855, 0.555)).norm(), 1e-12);

  const SE3 T_W_I_b(so3::Exp(Eigen::Vector3d(0, 0, M_PI / 2)), {10.0, -2.0, 0.5});
  const SE3 T_W_L_b = T_W_I_b * T_I_L;
  EXPECT_LT((T_W_L_b.t - Eigen::Vector3d(10.00855, -2.001, 0.555)).norm(), 1e-9);
  EXPECT_LT((T_W_L_b.t - (T_W_I_b.R * T_I_L.t + T_W_I_b.t)).norm(), 1e-12);
  // And back: T_W_I = T_W_L * T_I_L^-1.
  EXPECT_LT(poseDistance(T_W_L_b * T_I_L.inverse(), T_W_I_b), 1e-9);
}
